"""手动拖动低力矩助力的电机2，让电机1跟随其相对角度。

电机2：CAN ID 0x02 / Master ID 0x12，默认启用小幅摩擦补偿；
传入 --assist-torque 0 时保持完全失能，只读取编码器。
电机1：CAN ID 0x01 / Master ID 0x11，使能后采用受限速度闭环跟随。
两台电机启动时的位置分别作为相对零点，不会改写编码器零位。
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

from dmcan import DmCanContext, dmcan_device_type

from dm4310_keyboard import (
    VK_ESCAPE,
    clamp,
    key_is_down,
    unwrap_delta,
)
from dm4310_usb2fdcan import (
    CONTROL_PERIOD_S,
    Feedback,
    configure_classic_can_1m,
    pack_mit,
    parse_int,
    send,
    special,
    wait_until_enabled,
)


@dataclass
class RelativePosition:
    previous_raw: float
    value: float = 0.0

    def update(self, raw_position: float, direction: int) -> float:
        self.value += direction * unwrap_delta(raw_position, self.previous_raw)
        self.previous_raw = raw_position
        return self.value


def refresh_motor_status(device, channel: int, slave_id: int) -> None:
    """使用达妙状态刷新帧读取失能电机的编码器反馈。"""
    payload = bytes(
        [
            slave_id & 0xFF,
            (slave_id >> 8) & 0xFF,
            0xCC,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )
    send(device, channel, 0x7FF, payload)


def wait_for_disabled_feedback(
    device,
    feedback: Feedback,
    channel: int,
    slave_id: int,
    timeout: float = 1.0,
):
    deadline = time.monotonic() + timeout
    last_refresh = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_refresh >= 0.05:
            feedback.event.clear()
            refresh_motor_status(device, channel, slave_id)
            last_refresh = now
        if feedback.event.wait(0.02):
            state = feedback.snapshot()
            if state.status == 0x0:
                return state
            raise RuntimeError(
                f"电机2应处于失能状态，但反馈状态为 0x{state.status:X}"
            )
    raise RuntimeError("电机2失能后没有收到编码器反馈")


def checked_follower(feedback: Feedback, overspeed_limit: float):
    state = feedback.snapshot()
    now = time.monotonic()
    if not state.received_at or now - state.received_at > 0.5:
        raise RuntimeError("电机1超过 0.5 秒未收到反馈")
    if state.status != 0x1:
        raise RuntimeError(f"电机1状态异常：0x{state.status:X}")
    if abs(state.velocity) > overspeed_limit:
        raise RuntimeError(
            f"电机1触发超速保护：{state.velocity:.3f} rad/s"
        )
    return state


def checked_leader(
    feedback: Feedback,
    assisted: bool,
    overspeed_limit: float,
):
    state = feedback.snapshot()
    now = time.monotonic()
    if not state.received_at or now - state.received_at > 0.25:
        raise RuntimeError("电机2超过 0.25 秒未收到编码器反馈")
    expected_status = 0x1 if assisted else 0x0
    if state.status != expected_status:
        raise RuntimeError(
            f"电机2状态应为 0x{expected_status:X}，"
            f"实际为 0x{state.status:X}，已停止"
        )
    if assisted and abs(state.velocity) > overspeed_limit:
        raise RuntimeError(
            f"电机2助力状态下超速：{state.velocity:.3f} rad/s"
        )
    return state


def validate_args(args: argparse.Namespace) -> float:
    if args.slave_id_1 == args.slave_id_2:
        raise ValueError("两台电机的 CAN ID 不能相同")
    if args.master_id_1 == args.master_id_2:
        raise ValueError("两台电机的 Master ID 不能相同")
    if args.direction_1 not in (-1, 1) or args.direction_2 not in (-1, 1):
        raise ValueError("direction-1 和 direction-2 只能是 -1 或 1")
    if not 0 < args.max_speed <= 5.0:
        raise ValueError("max-speed 必须在 0..5 rad/s 之间")
    if not 0.02 <= args.ramp_time <= 2.0:
        raise ValueError("ramp-time 必须在 0.02..2 秒之间")
    if not 0 < args.position_gain <= 20.0:
        raise ValueError("position-gain 必须在 0..20 之间")
    if not 0 <= args.deadband <= 0.2:
        raise ValueError("deadband 必须在 0..0.2 rad 之间")
    if not 0 <= args.min_speed <= args.max_speed:
        raise ValueError("min-speed 必须在 0..max-speed 之间")
    if not 0 < args.position_limit < 12.5:
        raise ValueError("position-limit 必须在 0..12.5 rad 之间")
    if not 0 < abs(args.ratio) <= 5.0:
        raise ValueError("ratio 的绝对值必须在 0..5 之间")
    if not 0 <= args.kd <= 5.0:
        raise ValueError("kd 必须在 0..5 之间")
    if args.overspeed_limit <= args.max_speed:
        raise ValueError("overspeed-limit 必须大于 max-speed")
    if not 0 <= args.assist_torque <= 0.30:
        raise ValueError("assist-torque 必须在 0..0.30 N·m 之间")
    if not 0.10 <= args.assist_cutoff_speed <= 2.0:
        raise ValueError("assist-cutoff-speed 必须在 0.10..2 rad/s 之间")
    if not 0 <= args.assist_deadband < args.assist_cutoff_speed:
        raise ValueError("assist-deadband 必须小于 assist-cutoff-speed")
    return args.max_speed / args.ramp_time


def run(args: argparse.Namespace) -> None:
    acceleration = validate_args(args)
    context = None
    device = None
    channel_enabled = False
    follower_enabled = False
    leader_enabled = False

    try:
        context = DmCanContext()
        count = context.find_devices(dmcan_device_type.USB2CANFD)
        if count <= 0:
            raise RuntimeError(
                "没有找到 USB2FDCAN；请关闭网页服务、DMTool/USB2CAN 后重新插拔"
            )
        if args.device_index >= count:
            raise RuntimeError(f"只发现 {count} 个设备，device-index 超出范围")

        device = context.get_device(args.device_index)
        if not device.open():
            raise RuntimeError("USB2FDCAN 打开失败，设备可能被其他程序占用")

        feedback_1 = Feedback(
            args.slave_id_1,
            args.master_id_1,
            args.channel,
        )
        feedback_2 = Feedback(
            args.slave_id_2,
            args.master_id_2,
            args.channel,
        )

        def dispatch_feedback(callback_device, frame) -> None:
            feedback_1.callback(callback_device, frame)
            feedback_2.callback(callback_device, frame)

        device.hook_recv_callback(dispatch_feedback)
        configure_classic_can_1m(device, args.channel)
        channel_enabled = True

        # 启动时先明确失能两台；完成反馈检查后，再按所选模式准备电机2。
        special(device, args.channel, args.slave_id_1, 0xFD)
        special(device, args.channel, args.slave_id_2, 0xFD)
        time.sleep(0.2)

        leader_state = wait_for_disabled_feedback(
            device,
            feedback_2,
            args.channel,
            args.slave_id_2,
            timeout=1.0,
        )

        # 在失能状态给电机1预装零速阻尼，再单独使能电机1。
        zero_speed = pack_mit(0.0, 0.0, 0.0, args.kd, 0.0)
        for _ in range(3):
            send(
                device,
                args.channel,
                args.slave_id_1,
                zero_speed,
            )
            time.sleep(CONTROL_PERIOD_S)

        print(
            f"电机2已完成初始安全失能：Slave=0x{args.slave_id_2:02X}，"
            f"Master=0x{args.master_id_2:02X}"
        )
        print(
            f"使能电机1：Slave=0x{args.slave_id_1:02X}，"
            f"Master=0x{args.master_id_1:02X}"
        )
        follower_enabled = True
        follower_state = wait_until_enabled(
            device,
            feedback_1,
            args.channel,
            args.slave_id_1,
            timeout=1.0,
        )

        if args.assist_torque > 0:
            # 摩擦补偿需要功率级使能，但位置/速度增益均为零，只允许发送
            # 下方严格限幅的前馈力矩。
            leader_zero_torque = pack_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            for _ in range(3):
                send(
                    device,
                    args.channel,
                    args.slave_id_2,
                    leader_zero_torque,
                )
                time.sleep(CONTROL_PERIOD_S)
            print(
                f"使能电机2低力矩助力：最大补偿 "
                f"{args.assist_torque:.3f} N·m"
            )
            leader_enabled = True
            leader_state = wait_until_enabled(
                device,
                feedback_2,
                args.channel,
                args.slave_id_2,
                timeout=1.0,
            )
        else:
            leader_state = wait_for_disabled_feedback(
                device,
                feedback_2,
                args.channel,
                args.slave_id_2,
                timeout=1.0,
            )

        # 电机2模式准备期间刷新一次电机1的零速反馈，避免把旧时间戳误判
        # 为跟随电机通信中断。
        feedback_1.event.clear()
        send(
            device,
            args.channel,
            args.slave_id_1,
            zero_speed,
        )
        if not feedback_1.event.wait(0.2):
            raise RuntimeError("启动跟随前没有收到电机1零速反馈")
        follower_state = checked_follower(
            feedback_1,
            args.overspeed_limit,
        )

        leader_position = RelativePosition(leader_state.position)
        follower_position = RelativePosition(follower_state.position)
        velocity_command = 0.0
        last_print = time.monotonic()
        next_send = time.monotonic()
        refresh_counter = 0
        assist_torque_command = 0.0

        print()
        print("示教跟随已启动：手动拖动电机2，电机1将跟随相对角度")
        print(
            "电机2模式="
            + ("低力矩助力" if leader_enabled else "完全失能")
            + "；按 Esc 或 Ctrl+C 退出"
        )
        print(
            f"比例={args.ratio:+.2f}，最大速度={args.max_speed:.2f} rad/s，"
            f"响应={args.ramp_time:.3f} s，软限位=±{args.position_limit:.2f} rad"
        )

        while True:
            if not leader_enabled:
                # 失能电机不会周期主动反馈，发送状态刷新请求。部分固件不
                # 响应 0x7FF，低频重复失能命令作为后备。
                refresh_motor_status(
                    device,
                    args.channel,
                    args.slave_id_2,
                )
                refresh_counter += 1
                if refresh_counter % 10 == 0:
                    special(device, args.channel, args.slave_id_2, 0xFD)

            follower_state = checked_follower(
                feedback_1,
                args.overspeed_limit,
            )
            leader_state = checked_leader(
                feedback_2,
                leader_enabled,
                args.overspeed_limit,
            )
            leader_relative = leader_position.update(
                leader_state.position,
                args.direction_2,
            )
            follower_relative = follower_position.update(
                follower_state.position,
                args.direction_1,
            )

            target_position = clamp(
                args.ratio * leader_relative,
                args.position_limit,
            )
            error = target_position - follower_relative

            if leader_enabled:
                leader_velocity = args.direction_2 * leader_state.velocity
                if abs(leader_velocity) <= args.assist_deadband:
                    assist_torque_command = 0.0
                else:
                    # 低速时提供顺向助力，并随速度线性衰减；达到 cutoff
                    # 后补偿为零，避免补偿力矩维持电机自行旋转。
                    assist_scale = max(
                        0.0,
                        1.0
                        - abs(leader_velocity)
                        / args.assist_cutoff_speed,
                    )
                    assist_torque_command = math.copysign(
                        args.assist_torque * assist_scale,
                        leader_velocity,
                    )
                leader_command = pack_mit(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    args.direction_2 * assist_torque_command,
                )
                send(
                    device,
                    args.channel,
                    args.slave_id_2,
                    leader_command,
                )

            if key_is_down(VK_ESCAPE):
                print("\n收到 Esc，正在停止跟随并失能两台电机……")
                break

            if abs(error) <= args.deadband:
                desired_velocity = 0.0
            else:
                speed = min(
                    args.max_speed,
                    max(
                        args.min_speed,
                        args.position_gain * abs(error),
                    ),
                )
                desired_velocity = math.copysign(speed, error)

            # 接近相对软限位时提前限制速度，保证可在范围内刹停。
            if desired_velocity > 0:
                remaining = args.position_limit - follower_relative
                safe_speed = math.sqrt(
                    max(0.0, 2.0 * acceleration * remaining)
                )
                desired_velocity = min(desired_velocity, safe_speed)
            elif desired_velocity < 0:
                remaining = args.position_limit + follower_relative
                safe_speed = math.sqrt(
                    max(0.0, 2.0 * acceleration * remaining)
                )
                desired_velocity = max(desired_velocity, -safe_speed)

            velocity_command += clamp(
                desired_velocity - velocity_command,
                acceleration * CONTROL_PERIOD_S,
            )
            command = pack_mit(
                0.0,
                args.direction_1 * velocity_command,
                0.0,
                args.kd,
                0.0,
            )
            send(
                device,
                args.channel,
                args.slave_id_1,
                command,
            )

            now = time.monotonic()
            if now - last_print >= 0.05:
                print(
                    f"\r拖动={leader_relative:+7.3f} rad  "
                    f"目标={target_position:+7.3f} rad  "
                    f"电机1={follower_relative:+7.3f} rad  "
                    f"误差={error:+6.3f}  "
                    f"助力={assist_torque_command:+5.3f} N·m  "
                    f"速度={args.direction_1 * follower_state.velocity:+6.3f} rad/s",
                    end="",
                    flush=True,
                )
                last_print = now

            next_send += CONTROL_PERIOD_S
            delay = next_send - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_send = time.monotonic()

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停止并失能……")
    finally:
        if device is not None:
            if follower_enabled:
                stop_command = pack_mit(
                    0.0,
                    0.0,
                    0.0,
                    max(args.kd, 1.0),
                    0.0,
                )
                for _ in range(8):
                    try:
                        send(
                            device,
                            args.channel,
                            args.slave_id_1,
                            stop_command,
                        )
                    except Exception:
                        break
                    time.sleep(CONTROL_PERIOD_S)

            # 无论前面发生什么，都分别尝试失能两台电机。
            for _ in range(3):
                for slave_id in (args.slave_id_1, args.slave_id_2):
                    try:
                        special(device, args.channel, slave_id, 0xFD)
                    except Exception:
                        pass
                time.sleep(0.01)
            print("\n两台电机已失能")

            if channel_enabled:
                try:
                    device.enable_channel(args.channel, False)
                except Exception:
                    pass
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
    parser = argparse.ArgumentParser(
        description="拖动低力矩助力（或完全失能）的电机2控制电机1"
    )
    parser.add_argument("--slave-id-1", type=parse_int, default=0x01)
    parser.add_argument("--master-id-1", type=parse_int, default=0x11)
    parser.add_argument("--slave-id-2", type=parse_int, default=0x02)
    parser.add_argument("--master-id-2", type=parse_int, default=0x12)
    parser.add_argument("--direction-1", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--direction-2", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--ratio", type=float, default=1.0)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--max-speed", type=float, default=2.5)
    parser.add_argument("--ramp-time", type=float, default=0.10)
    parser.add_argument("--position-gain", type=float, default=6.0)
    parser.add_argument("--deadband", type=float, default=0.01)
    parser.add_argument("--min-speed", type=float, default=0.12)
    parser.add_argument("--position-limit", type=float, default=3.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--overspeed-limit", type=float, default=4.0)
    parser.add_argument(
        "--assist-torque",
        type=float,
        default=0.10,
        help="电机2低速拖动时的最大顺向补偿力矩 N·m；0 表示保持失能",
    )
    parser.add_argument(
        "--assist-cutoff-speed",
        type=float,
        default=0.60,
        help="电机2达到该速度时将助力衰减到零 rad/s",
    )
    parser.add_argument(
        "--assist-deadband",
        type=float,
        default=0.03,
        help="低于该速度不输出助力 rad/s",
    )
    return parser


if __name__ == "__main__":
    run(argument_parser().parse_args())
