"""使用 Windows 键盘左右方向键实时控制 DM-J4310 转动。

按住 ←/→ 转动，松开后平滑减速，空格制动，Esc 退出并失能。
默认以启动位置为中心限制在 ±3 rad，防止无意中持续旋转。
"""

from __future__ import annotations

import argparse
import ctypes
import math
import time

from dmcan import DmCanContext, dmcan_device_type

from dm4310_usb2fdcan import (
    CONTROL_PERIOD_S,
    P_MAX,
    Feedback,
    configure_classic_can_1m,
    pack_mit,
    parse_int,
    send,
    special,
    wait_until_enabled,
)


VK_LEFT = 0x25
VK_UP = 0x26
VK_RIGHT = 0x27
VK_DOWN = 0x28
VK_SPACE = 0x20
VK_ESCAPE = 0x1B
POSITION_PERIOD = 2.0 * P_MAX
MAX_KEY_SPEED = 5.0
MIN_KEY_SPEED = 0.5


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def key_is_down(virtual_key: int) -> bool:
    """读取 Windows 键的实时物理按下状态，而不是依赖按键重复事件。"""
    return bool(ctypes.windll.user32.GetAsyncKeyState(virtual_key) & 0x8000)


def unwrap_delta(current: float, previous: float) -> float:
    """消除 MIT 位置反馈在 ±P_MAX 处的回绕。"""
    delta = current - previous
    if delta > P_MAX:
        delta -= POSITION_PERIOD
    elif delta < -P_MAX:
        delta += POSITION_PERIOD
    return delta


def checked_state(feedback: Feedback, overspeed_limit: float):
    state = feedback.snapshot()
    now = time.monotonic()
    if not state.received_at or now - state.received_at > 0.5:
        raise RuntimeError("超过 0.5 秒未收到电机反馈")
    if state.status != 0x1:
        raise RuntimeError(f"电机状态异常：0x{state.status:X}")
    if abs(state.velocity) > overspeed_limit:
        raise RuntimeError(
            f"触发超速保护：实测 {state.velocity:.3f} rad/s，"
            f"限制 {overspeed_limit:.3f} rad/s"
        )
    return state


def run(args: argparse.Namespace) -> None:
    if args.direction not in (-1, 1):
        raise ValueError("direction 只能是 -1 或 1")
    if not 0 < args.speed <= MAX_KEY_SPEED:
        raise ValueError("speed 必须在 0..5 rad/s 之间")
    if not 0.02 <= args.ramp_time <= 2.0:
        raise ValueError("ramp-time 必须在 0.02..2 秒之间")
    initial_acceleration = (
        args.acceleration if args.acceleration is not None else args.speed / args.ramp_time
    )
    if not 0 < initial_acceleration <= 100.0:
        raise ValueError("acceleration 必须在 0..100 rad/s² 之间")
    if not 0 < args.speed_step <= 2.0:
        raise ValueError("speed-step 必须在 0..2 rad/s 之间")
    if not 0 <= args.kd <= 5.0:
        raise ValueError("kd 必须在 0..5 之间")
    if args.position_limit <= 0 or args.position_limit >= P_MAX:
        raise ValueError(f"position-limit 必须在 0..{P_MAX:g} rad 之间")
    if args.overspeed_limit <= MAX_KEY_SPEED:
        raise ValueError("overspeed-limit 必须大于键盘速度上限 5 rad/s")

    context = None
    device = None
    enabled = False
    channel_enabled = False

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

        feedback = Feedback(args.slave_id, args.master_id, args.channel)
        device.hook_recv_callback(feedback.callback)
        configure_classic_can_1m(device, args.channel)
        channel_enabled = True

        # 从安全失能状态开始，并预装零速阻尼命令。
        special(device, args.channel, args.slave_id, 0xFD)
        time.sleep(0.2)
        zero_speed = pack_mit(0.0, 0.0, 0.0, args.kd, 0.0)
        for _ in range(3):
            send(device, args.channel, args.slave_id, zero_speed)
            time.sleep(CONTROL_PERIOD_S)

        print(
            f"USB2FDCAN 已连接：CH{args.channel}，Slave=0x{args.slave_id:02X}，"
            f"Master=0x{args.master_id:02X}"
        )
        print("发送使能命令……")
        enabled = True
        state = wait_until_enabled(
            device,
            feedback,
            args.channel,
            args.slave_id,
            timeout=1.0,
        )

        previous_raw_position = state.position
        relative_position = 0.0
        operator_velocity_command = 0.0
        current_speed = args.speed
        previous_up = False
        previous_down = False
        last_print = time.monotonic()
        next_send = time.monotonic()

        print()
        print("控制键：按住 ← / → 转动，↑ / ↓ 调速，空格制动，Esc 退出")
        print(
            f"速度={args.speed:.2f} rad/s，响应时间={args.ramp_time:.3f} s，"
            f"指令加速度={initial_acceleration:.2f} rad/s²，"
            f"相对位置限制=±{args.position_limit:.2f} rad，方向={args.direction:+d}"
        )
        print("请保持当前终端窗口处于前台。")

        while True:
            state = checked_state(feedback, args.overspeed_limit)
            motor_delta = unwrap_delta(state.position, previous_raw_position)
            previous_raw_position = state.position
            relative_position += args.direction * motor_delta

            if key_is_down(VK_ESCAPE):
                print("\n收到 Esc，正在停止并失能……")
                break

            left_down = key_is_down(VK_LEFT)
            right_down = key_is_down(VK_RIGHT)
            up_down = key_is_down(VK_UP)
            down_down = key_is_down(VK_DOWN)
            brake_down = key_is_down(VK_SPACE)

            # 上下键只在按下边沿调整一次，避免按住时每 10 ms 连续跳变。
            if up_down and not previous_up and not down_down:
                current_speed = min(
                    MAX_KEY_SPEED,
                    current_speed + args.speed_step,
                )
                print(f"\n速度上调至 {current_speed:.2f} rad/s")
            elif down_down and not previous_down and not up_down:
                current_speed = max(
                    MIN_KEY_SPEED,
                    current_speed - args.speed_step,
                )
                print(f"\n速度下调至 {current_speed:.2f} rad/s")
            previous_up = up_down
            previous_down = down_down

            acceleration = (
                args.acceleration
                if args.acceleration is not None
                else current_speed / args.ramp_time
            )

            if brake_down:
                desired_operator_velocity = 0.0
                operator_velocity_command = 0.0
            elif left_down == right_down:
                desired_operator_velocity = 0.0
            elif left_down:
                desired_operator_velocity = -current_speed
            else:
                desired_operator_velocity = current_speed

            # 根据剩余距离限制接近软限位时的速度，确保能平滑刹停。
            if desired_operator_velocity > 0:
                remaining = args.position_limit - relative_position
                safe_speed = math.sqrt(
                    max(0.0, 2.0 * acceleration * remaining)
                )
                desired_operator_velocity = min(
                    desired_operator_velocity,
                    safe_speed,
                )
            elif desired_operator_velocity < 0:
                remaining = args.position_limit + relative_position
                safe_speed = math.sqrt(
                    max(0.0, 2.0 * acceleration * remaining)
                )
                desired_operator_velocity = max(
                    desired_operator_velocity,
                    -safe_speed,
                )

            operator_velocity_command += clamp(
                desired_operator_velocity - operator_velocity_command,
                acceleration * CONTROL_PERIOD_S,
            )
            motor_velocity_command = args.direction * operator_velocity_command
            command = pack_mit(
                0.0,
                motor_velocity_command,
                0.0,
                args.kd,
                0.0,
            )
            send(device, args.channel, args.slave_id, command)

            now = time.monotonic()
            if now - last_print >= 0.10:
                key_text = (
                    "制动"
                    if brake_down
                    else "左转"
                    if left_down and not right_down
                    else "右转"
                    if right_down and not left_down
                    else "停止"
                )
                operator_velocity = args.direction * state.velocity
                print(
                    f"\r[{key_text:2}] 相对位置={relative_position:+7.3f} rad  "
                    f"设定={current_speed:4.1f}  "
                    f"指令速度={operator_velocity_command:+6.3f} rad/s  "
                    f"实际速度={operator_velocity:+6.3f} rad/s  "
                    f"力矩={args.direction * state.torque:+6.3f} N·m",
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
        print("\n收到 Ctrl+C，正在安全停止……")
    finally:
        if device is not None:
            try:
                if enabled:
                    # 先发送短暂零速阻尼，再发失能命令。
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
                                args.slave_id,
                                stop_command,
                            )
                        except Exception:
                            break
                        time.sleep(CONTROL_PERIOD_S)
                for _ in range(3):
                    try:
                        special(device, args.channel, args.slave_id, 0xFD)
                    except Exception:
                        break
                    time.sleep(0.01)
                print("\n电机已失能")
            finally:
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
    parser = argparse.ArgumentParser(description="DM-J4310 键盘左右转动控制")
    parser.add_argument("--slave-id", type=parse_int, default=0x01)
    parser.add_argument("--master-id", type=parse_int, default=0x11)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--speed", type=float, default=2.50, help="按键目标速度 rad/s")
    parser.add_argument(
        "--speed-step",
        type=float,
        default=0.50,
        help="每次按上下键的速度调整量 rad/s",
    )
    parser.add_argument(
        "--ramp-time",
        type=float,
        default=0.10,
        help="按下加速、松开减速所需时间（秒）",
    )
    parser.add_argument(
        "--acceleration",
        type=float,
        default=None,
        help="直接指定加速度 rad/s²；设置后覆盖 ramp-time 的换算值",
    )
    parser.add_argument("--kd", type=float, default=1.0, help="MIT 速度阻尼 0..5")
    parser.add_argument(
        "--direction",
        type=int,
        choices=(-1, 1),
        default=-1,
        help="键盘方向到电机方向的映射，当前机构默认 -1",
    )
    parser.add_argument(
        "--position-limit",
        type=float,
        default=3.0,
        help="相对启动位置的左右软限位 rad",
    )
    parser.add_argument(
        "--overspeed-limit",
        type=float,
        default=6.0,
        help="实测速度超过该值立即失能 rad/s",
    )
    return parser


if __name__ == "__main__":
    run(argument_parser().parse_args())
