"""让 DM-J4310 通过受限速度闭环缓慢移动到坐标系的 0 rad。

这只执行位置运动，不会改写或保存电机编码器零点。运行前必须完全关闭
DMTool/USB2CAN 图形程序。程序正常结束、异常或 Ctrl+C 时都会失能电机。

MIT 位置反馈只在 -12.5..12.5 rad 内编码，累计位置越界时会回绕，因此
归零阶段不能把首次反馈直接作为绝对位置目标。本脚本使用速度闭环接近零位，
仅在已经进入零点容差后才启用低增益位置保持。
"""

from __future__ import annotations

import argparse
import math
import time

from dmcan import DmCanContext, dmcan_device_type

from dm4310_usb2fdcan import (
    CONTROL_PERIOD_S,
    Feedback,
    configure_classic_can_1m,
    pack_mit,
    parse_int,
    send,
    smoothstep,
    special,
    wait_until_enabled,
)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def check_feedback(feedback: Feedback) -> None:
    state = feedback.snapshot()
    now = time.monotonic()
    if not state.received_at or now - state.received_at > 0.5:
        raise RuntimeError("超过 0.5 秒未收到电机反馈")
    if state.status != 0x1:
        raise RuntimeError(
            f"运行中电机退出使能状态，状态码 0x{state.status:X}"
        )


def run(args: argparse.Namespace) -> None:
    if args.duration <= 0:
        raise ValueError("duration 必须大于 0")
    if not 0.0 <= args.kp <= 20.0:
        raise ValueError("kp 必须在 0..20 之间")
    if not 0.0 <= args.kd <= 5.0:
        raise ValueError("kd 必须在 0..5 之间")
    if args.hold_time < 0:
        raise ValueError("hold-time 不能小于 0")
    if args.tolerance <= 0:
        raise ValueError("tolerance 必须大于 0")
    if args.max_speed <= 0:
        raise ValueError("max-speed 必须大于 0")
    if args.max_acceleration <= 0:
        raise ValueError("max-acceleration 必须大于 0")
    if not 0 < args.min_speed <= args.max_speed:
        raise ValueError("min-speed 必须大于 0 且不超过 max-speed")
    if args.velocity_tolerance <= 0:
        raise ValueError("velocity-tolerance 必须大于 0")
    if args.settle_time < 0:
        raise ValueError("settle-time 不能小于 0")
    if args.overspeed_limit <= args.max_speed:
        raise ValueError("overspeed-limit 必须大于 max-speed")
    if args.capture_radius <= args.tolerance:
        raise ValueError("capture-radius 必须大于 tolerance")
    if args.capture_exit <= args.capture_radius:
        raise ValueError("capture-exit 必须大于 capture-radius")
    if args.capture_ramp_time <= 0:
        raise ValueError("capture-ramp-time 必须大于 0")
    if args.progress_timeout <= 0:
        raise ValueError("progress-timeout 必须大于 0")
    if args.progress_epsilon <= 0:
        raise ValueError("progress-epsilon 必须大于 0")
    if args.capture_ki < 0:
        raise ValueError("capture-ki 不能小于 0")
    if not 0 < args.integral_torque_limit <= 2.0:
        raise ValueError("integral-torque-limit 必须在 0..2 N·m 之间")

    context = None
    device = None
    enabled = False
    channel_enabled = False

    try:
        context = DmCanContext()
        count = context.find_devices(dmcan_device_type.USB2CANFD)
        if count <= 0:
            raise RuntimeError(
                "没有找到可用的 USB2FDCAN。请完全退出 DMTool/USB2CAN，"
                "重新插拔设备后再运行"
            )
        if args.device_index >= count:
            raise RuntimeError(f"只发现 {count} 个设备，device-index 超出范围")

        device = context.get_device(args.device_index)
        if not device.open():
            raise RuntimeError("USB2FDCAN 打开失败，设备可能被 DMTool 占用")

        feedback = Feedback(args.slave_id, args.master_id, args.channel)
        device.hook_recv_callback(feedback.callback)

        try:
            configure_classic_can_1m(device, args.channel)
            channel_enabled = True
            print(
                f"USB2FDCAN 已连接：CH{args.channel}，经典 CAN 1M，"
                f"Slave=0x{args.slave_id:02X}，Master=0x{args.master_id:02X}"
            )

            # 从已知的安全失能状态开始，并给延迟反馈留出接收时间。
            special(device, args.channel, args.slave_id, 0xFD)
            time.sleep(0.2)
            feedback.event.clear()

            # 在失能状态预装零速阻尼帧，避免使能瞬间执行驱动器中残留的
            # 上一条位置/速度命令。部分固件会保存最近一次 MIT 目标值。
            preload = pack_mit(0.0, 0.0, 0.0, args.kd, 0.0)
            for _ in range(3):
                send(device, args.channel, args.slave_id, preload)
                time.sleep(CONTROL_PERIOD_S)

            print("发送使能命令……")
            enabled = True
            state = wait_until_enabled(
                device,
                feedback,
                args.channel,
                args.slave_id,
                timeout=1.0,
            )
            start_position = state.position
            print(
                f"当前位置={start_position:.4f} rad，开始限速归零："
                f"最大速度={args.max_speed:.3f} rad/s，"
                f"最大加速度={args.max_acceleration:.3f} rad/s²"
            )

            start = time.monotonic()
            next_send = start
            last_print = start
            velocity_command = 0.0
            settled_since = None
            mode = "approach"
            capture_started = None
            best_error = abs(start_position)
            last_progress = start
            command_kp = 0.0
            integral_torque = 0.0
            while time.monotonic() - start < args.duration:
                elapsed = time.monotonic() - start
                check_feedback(feedback)
                state = feedback.snapshot()
                now = time.monotonic()

                if abs(state.velocity) > args.overspeed_limit:
                    raise RuntimeError(
                        f"触发超速保护：实测 {state.velocity:.3f} rad/s，"
                        f"限制 {args.overspeed_limit:.3f} rad/s"
                    )

                position_error = abs(state.position)
                if position_error < best_error - args.progress_epsilon:
                    best_error = position_error
                    last_progress = now

                if mode == "approach" and position_error <= args.capture_radius:
                    mode = "capture"
                    capture_started = now
                    settled_since = None
                    integral_torque = 0.0
                    print(
                        f"进入位置捕获区：|位置|={position_error:.4f} rad，"
                        "开始逐渐增加位置刚度"
                    )

                if mode == "capture" and position_error > args.capture_exit:
                    mode = "approach"
                    capture_started = None
                    settled_since = None
                    velocity_command = 0.0
                    integral_torque = 0.0
                    print("离开位置捕获区，退回限速接近阶段")

                if mode == "approach":
                    if now - last_progress > args.progress_timeout:
                        raise RuntimeError(
                            f"归零无进展：{args.progress_timeout:.1f} 秒内位置误差"
                            f"未明显减小，当前位置={state.position:.4f} rad；"
                            "请检查机械卡滞、方向或适当增加 min-speed/KD"
                        )

                    # 根据剩余制动距离计算速度；离捕获区较远时保留最小速度，
                    # 克服减速器静摩擦，接近捕获区时再由位置环接管。
                    braking_distance = max(
                        position_error - args.capture_radius,
                        0.0,
                    )
                    braking_speed = math.sqrt(
                        2.0 * args.max_acceleration * braking_distance
                    )
                    speed_magnitude = min(
                        args.max_speed,
                        max(args.min_speed, braking_speed),
                    )
                    desired_velocity = math.copysign(
                        speed_magnitude,
                        -state.position,
                    )
                    maximum_step = args.max_acceleration * CONTROL_PERIOD_S
                    velocity_command += clamp(
                        desired_velocity - velocity_command,
                        maximum_step,
                    )
                    command_kp = 0.0
                    command = pack_mit(
                        0.0,
                        velocity_command,
                        0.0,
                        args.kd,
                        0.0,
                    )
                else:
                    velocity_command = 0.0
                    ramp_ratio = (now - capture_started) / args.capture_ramp_time
                    command_kp = args.kp * smoothstep(ramp_ratio)

                    # 位置比例项可能无法克服减速器静摩擦。对位置误差进行
                    # 小幅、限速、限幅积分，并作为 MIT 前馈力矩补偿。
                    # 误差过零后积分方向自然反转，避免永久单向偏置。
                    integral_torque += (
                        args.capture_ki
                        * (-state.position)
                        * CONTROL_PERIOD_S
                    )
                    integral_torque = clamp(
                        integral_torque,
                        args.integral_torque_limit,
                    )
                    command = pack_mit(
                        0.0,
                        0.0,
                        command_kp,
                        args.kd,
                        integral_torque,
                    )

                    if (
                        position_error <= args.tolerance
                        and abs(state.velocity) <= args.velocity_tolerance
                    ):
                        if settled_since is None:
                            settled_since = now
                        elif now - settled_since >= args.settle_time:
                            break
                    else:
                        settled_since = None

                send(device, args.channel, args.slave_id, command)

                if now - last_print >= 0.25:
                    print(
                        f"阶段={'接近' if mode == 'approach' else '捕获'}  "
                        f"位置={state.position:8.4f} rad  "
                        f"目标速度={velocity_command:7.3f} rad/s  "
                        f"KP={command_kp:5.2f}  "
                        f"Tff={integral_torque:6.3f} N·m  "
                        f"速度={state.velocity:7.3f} rad/s  "
                        f"转矩={state.torque:7.3f} N·m"
                    )
                    last_print = now

                next_send += CONTROL_PERIOD_S
                delay = next_send - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_send = time.monotonic()
            else:
                state = feedback.snapshot()
                raise RuntimeError(
                    f"在 {args.duration:.1f} 秒内未归零，"
                    f"当前位置={state.position:.4f} rad；"
                    "可增加 duration，但不要提高安全速度后盲目重试"
                )

            print(f"零位已捕获，在零位保持 {args.hold_time:.2f} 秒……")
            zero_position = pack_mit(
                0.0,
                0.0,
                args.kp,
                args.kd,
                integral_torque,
            )
            send(device, args.channel, args.slave_id, zero_position)
            hold_deadline = time.monotonic() + args.hold_time
            while time.monotonic() < hold_deadline:
                check_feedback(feedback)
                state = feedback.snapshot()
                if abs(state.velocity) > args.overspeed_limit:
                    raise RuntimeError(
                        f"保持阶段触发超速保护：{state.velocity:.3f} rad/s"
                    )
                integral_torque += (
                    args.capture_ki
                    * (-state.position)
                    * CONTROL_PERIOD_S
                )
                integral_torque = clamp(
                    integral_torque,
                    args.integral_torque_limit,
                )
                zero_position = pack_mit(
                    0.0,
                    0.0,
                    args.kp,
                    args.kd,
                    integral_torque,
                )
                send(device, args.channel, args.slave_id, zero_position)
                time.sleep(CONTROL_PERIOD_S)

            state = feedback.snapshot()
            error = abs(state.position)
            print(
                f"归零结束：位置={state.position:.4f} rad，"
                f"误差={error:.4f} rad"
            )
            if error > args.tolerance:
                raise RuntimeError(
                    f"最终位置误差 {error:.4f} rad 超过允许值 "
                    f"{args.tolerance:.4f} rad；可适当增加 hold-time 或 KP"
                )

        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，正在安全停止……")
        finally:
            if device is not None:
                try:
                    # 先短暂发送零速度阻尼命令，再失能。
                    if enabled:
                        zero_speed = pack_mit(
                            0.0, 0.0, 0.0, max(args.kd, 1.0), 0.0
                        )
                        for _ in range(5):
                            send(
                                device,
                                args.channel,
                                args.slave_id,
                                zero_speed,
                            )
                            time.sleep(CONTROL_PERIOD_S)
                    for _ in range(3):
                        special(device, args.channel, args.slave_id, 0xFD)
                        time.sleep(0.01)
                    print("电机已失能")
                finally:
                    if channel_enabled:
                        device.enable_channel(args.channel, False)
    finally:
        # dmcan-sdk 1.0.4 在 Windows 清理异步传输时可能产生无害提示。
        if device is not None:
            try:
                device.close()
            except OSError as exc:
                print(f"提示：USB2FDCAN 关闭设备时返回 {exc}，可忽略")
        if context is not None:
            try:
                context.destroy()
            except OSError as exc:
                print(f"提示：USB2FDCAN SDK 清理句柄时返回 {exc}，可忽略")
                context._ctx = None


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="DM-J4310 缓慢移动到 0 rad")
    parser.add_argument("--slave-id", type=parse_int, default=0x01)
    parser.add_argument("--master-id", type=parse_int, default=0x11)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="允许的最大归零时间（秒）",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=0.5,
        help="归零轨迹的最大目标速度（rad/s）",
    )
    parser.add_argument(
        "--max-acceleration",
        type=float,
        default=0.5,
        help="速度指令的最大变化率（rad/s²）",
    )
    parser.add_argument(
        "--min-speed",
        type=float,
        default=0.12,
        help="接近阶段用于克服静摩擦的最低目标速度（rad/s）",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=2.0,
        help="进入零点后的位置保持增益 0..20",
    )
    parser.add_argument("--kd", type=float, default=1.0, help="速度阻尼 0..5")
    parser.add_argument(
        "--hold-time",
        type=float,
        default=2.0,
        help="到达零位后的保持时间（秒）",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.005,
        help="允许的最终位置误差（rad）",
    )
    parser.add_argument(
        "--velocity-tolerance",
        type=float,
        default=0.03,
        help="判定停稳的速度阈值（rad/s）",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.5,
        help="进入零点后需要连续稳定的时间（秒）",
    )
    parser.add_argument(
        "--overspeed-limit",
        type=float,
        default=2.0,
        help="实测速度超过该值立即停止并失能（rad/s）",
    )
    parser.add_argument(
        "--capture-radius",
        type=float,
        default=0.20,
        help="切换到低增益位置捕获的距离（rad）",
    )
    parser.add_argument(
        "--capture-exit",
        type=float,
        default=0.45,
        help="位置捕获失败并退回速度接近的距离（rad）",
    )
    parser.add_argument(
        "--capture-ramp-time",
        type=float,
        default=1.0,
        help="位置捕获阶段 KP 从零增加到设定值的时间（秒）",
    )
    parser.add_argument(
        "--capture-ki",
        type=float,
        default=1.0,
        help="捕获阶段消除静摩擦稳态误差的积分增益",
    )
    parser.add_argument(
        "--integral-torque-limit",
        type=float,
        default=0.35,
        help="积分前馈力矩绝对值上限（N·m）",
    )
    parser.add_argument(
        "--progress-timeout",
        type=float,
        default=3.0,
        help="位置误差长期不减小时停止的等待时间（秒）",
    )
    parser.add_argument(
        "--progress-epsilon",
        type=float,
        default=0.02,
        help="判定位置取得进展的最小变化量（rad）",
    )
    return parser


if __name__ == "__main__":
    run(argument_parser().parse_args())
