"""通过同一 USB2FDCAN 总线独立键盘控制两台 DM-J4310。

默认电机 1：CAN ID 0x01、Master ID 0x11；
默认电机 2：CAN ID 0x02、Master ID 0x12。
任一电机发生通信、状态或超速异常时，两台电机都会一起失能。
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

from dmcan import DmCanContext, dmcan_device_type

from dm4310_keyboard import (
    MAX_KEY_SPEED,
    VK_DOWN,
    VK_ESCAPE,
    VK_LEFT,
    VK_RIGHT,
    VK_SPACE,
    VK_UP,
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
class MotorRuntime:
    name: str
    slave_id: int
    master_id: int
    direction: int
    feedback: Feedback
    previous_raw_position: float = 0.0
    relative_position: float = 0.0


def checked_state(motor: MotorRuntime, overspeed_limit: float):
    state = motor.feedback.snapshot()
    now = time.monotonic()
    if not state.received_at or now - state.received_at > 0.5:
        raise RuntimeError(f"{motor.name} 超过 0.5 秒未收到反馈")
    if state.status != 0x1:
        raise RuntimeError(f"{motor.name} 状态异常：0x{state.status:X}")
    if abs(state.velocity) > overspeed_limit:
        raise RuntimeError(
            f"{motor.name} 触发超速保护：{state.velocity:.3f} rad/s"
        )
    return state


def validate_args(args: argparse.Namespace) -> float:
    if args.direction_1 not in (-1, 1) or args.direction_2 not in (-1, 1):
        raise ValueError("direction-1 和 direction-2 只能是 -1 或 1")
    if args.slave_id_1 == args.slave_id_2:
        raise ValueError("两台电机的 CAN ID 不能相同")
    if args.master_id_1 == args.master_id_2:
        raise ValueError("两台电机的 Master ID 不能相同")
    if not 0 < args.speed <= MAX_KEY_SPEED:
        raise ValueError("speed 必须在 0..5 rad/s 之间")
    if not 0.02 <= args.ramp_time <= 2.0:
        raise ValueError("ramp-time 必须在 0.02..2 秒之间")
    if not 0 <= args.kd <= 5.0:
        raise ValueError("kd 必须在 0..5 之间")
    if not 0 < args.position_limit < 12.5:
        raise ValueError("position-limit 必须在 0..12.5 rad 之间")
    if args.overspeed_limit <= MAX_KEY_SPEED:
        raise ValueError("overspeed-limit 必须大于键盘速度上限 5 rad/s")

    acceleration = (
        args.acceleration
        if args.acceleration is not None
        else args.speed / args.ramp_time
    )
    if not 0 < acceleration <= 100.0:
        raise ValueError("acceleration 必须在 0..100 rad/s² 之间")
    return acceleration


def run(args: argparse.Namespace) -> None:
    initial_acceleration = validate_args(args)
    context = None
    device = None
    channel_enabled = False
    enabled_motors: list[MotorRuntime] = []

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
        motors = [
            MotorRuntime(
                "电机1",
                args.slave_id_1,
                args.master_id_1,
                args.direction_1,
                feedback_1,
            ),
            MotorRuntime(
                "电机2",
                args.slave_id_2,
                args.master_id_2,
                args.direction_2,
                feedback_2,
            ),
        ]

        def dispatch_feedback(callback_device, frame) -> None:
            feedback_1.callback(callback_device, frame)
            feedback_2.callback(callback_device, frame)

        device.hook_recv_callback(dispatch_feedback)
        configure_classic_can_1m(device, args.channel)
        channel_enabled = True

        # 两台电机全部从失能、零速阻尼状态开始。
        for motor in motors:
            special(device, args.channel, motor.slave_id, 0xFD)
        time.sleep(0.2)
        zero_speed = pack_mit(0.0, 0.0, 0.0, args.kd, 0.0)
        for _ in range(3):
            for motor in motors:
                send(
                    device,
                    args.channel,
                    motor.slave_id,
                    zero_speed,
                )
            time.sleep(CONTROL_PERIOD_S)

        print("USB2FDCAN 已连接：CH0，经典 CAN 1 Mbps")
        for motor in motors:
            print(
                f"使能{motor.name}：Slave=0x{motor.slave_id:02X}，"
                f"Master=0x{motor.master_id:02X}，方向={motor.direction:+d}"
            )
            state = wait_until_enabled(
                device,
                motor.feedback,
                args.channel,
                motor.slave_id,
                timeout=1.0,
            )
            motor.previous_raw_position = state.position
            enabled_motors.append(motor)

        # 确认两台均已进入使能状态后才允许键盘运动。
        for motor in motors:
            checked_state(motor, args.overspeed_limit)

        operator_velocity_commands = [0.0, 0.0]
        last_print = time.monotonic()
        next_send = time.monotonic()

        print()
        print("两台电机已就绪：←/→ 控制电机1，↑/↓ 控制电机2")
        print("空格立即制动，Esc 或 Ctrl+C 同时失能两台电机")
        print(
            f"速度={args.speed:.2f} rad/s，响应={args.ramp_time:.3f} s，"
            f"初始加速度={initial_acceleration:.2f} rad/s²，"
            f"各自相对软限位=±{args.position_limit:.2f} rad"
        )

        while True:
            states = [
                checked_state(motor, args.overspeed_limit)
                for motor in motors
            ]
            for motor, state in zip(motors, states):
                motor_delta = unwrap_delta(
                    state.position,
                    motor.previous_raw_position,
                )
                motor.previous_raw_position = state.position
                motor.relative_position += motor.direction * motor_delta

            if key_is_down(VK_ESCAPE):
                print("\n收到 Esc，正在同时停止两台电机……")
                break

            left_down = key_is_down(VK_LEFT)
            right_down = key_is_down(VK_RIGHT)
            up_down = key_is_down(VK_UP)
            down_down = key_is_down(VK_DOWN)
            brake_down = key_is_down(VK_SPACE)

            acceleration = (
                args.acceleration
                if args.acceleration is not None
                else args.speed / args.ramp_time
            )

            if brake_down:
                desired_operator_velocities = [0.0, 0.0]
                operator_velocity_commands = [0.0, 0.0]
            else:
                motor_1_velocity = (
                    0.0
                    if left_down == right_down
                    else -args.speed
                    if left_down
                    else args.speed
                )
                motor_2_velocity = (
                    0.0
                    if up_down == down_down
                    else args.speed
                    if up_down
                    else -args.speed
                )
                desired_operator_velocities = [
                    motor_1_velocity,
                    motor_2_velocity,
                ]

            # 两台电机分别根据自己的相对位置计算软限位制动速度。
            for index, (motor, desired_velocity) in enumerate(
                zip(motors, desired_operator_velocities)
            ):
                if desired_velocity > 0:
                    remaining = args.position_limit - motor.relative_position
                    safe_speed = math.sqrt(
                        max(0.0, 2.0 * acceleration * remaining)
                    )
                    desired_velocity = min(desired_velocity, safe_speed)
                elif desired_velocity < 0:
                    remaining = args.position_limit + motor.relative_position
                    safe_speed = math.sqrt(
                        max(0.0, 2.0 * acceleration * remaining)
                    )
                    desired_velocity = max(desired_velocity, -safe_speed)

                operator_velocity_commands[index] += clamp(
                    desired_velocity - operator_velocity_commands[index],
                    acceleration * CONTROL_PERIOD_S,
                )

            for motor, velocity_command in zip(
                motors,
                operator_velocity_commands,
            ):
                command = pack_mit(
                    0.0,
                    motor.direction * velocity_command,
                    0.0,
                    args.kd,
                    0.0,
                )
                send(
                    device,
                    args.channel,
                    motor.slave_id,
                    command,
                )

            now = time.monotonic()
            if now - last_print >= 0.10:
                key_1 = "左" if left_down and not right_down else "右" if right_down and not left_down else "停"
                key_2 = "上" if up_down and not down_down else "下" if down_down and not up_down else "停"
                if brake_down:
                    key_1 = key_2 = "刹"
                print(
                    f"\r[M1:{key_1} M2:{key_2}]  "
                    f"P1={motors[0].relative_position:+6.2f} "
                    f"V1={motors[0].direction * states[0].velocity:+5.2f}  "
                    f"C1={operator_velocity_commands[0]:+5.2f}  "
                    f"P2={motors[1].relative_position:+6.2f} "
                    f"V2={motors[1].direction * states[1].velocity:+5.2f}  "
                    f"C2={operator_velocity_commands[1]:+5.2f} rad/s",
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
        print("\n收到 Ctrl+C，正在同时停止两台电机……")
    finally:
        if device is not None:
            stop_command = pack_mit(0.0, 0.0, 0.0, max(args.kd, 1.0), 0.0)
            for _ in range(8):
                for motor in enabled_motors:
                    try:
                        send(
                            device,
                            args.channel,
                            motor.slave_id,
                            stop_command,
                        )
                    except Exception:
                        pass
                time.sleep(CONTROL_PERIOD_S)

            # 即使其中一个发送失败，也继续尝试失能另一个。
            for _ in range(3):
                for motor in enabled_motors:
                    try:
                        special(
                            device,
                            args.channel,
                            motor.slave_id,
                            0xFD,
                        )
                    except Exception:
                        pass
                time.sleep(0.01)
            if enabled_motors:
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
    parser = argparse.ArgumentParser(description="双 DM-J4310 独立键盘控制")
    parser.add_argument("--slave-id-1", type=parse_int, default=0x01)
    parser.add_argument("--master-id-1", type=parse_int, default=0x11)
    parser.add_argument("--slave-id-2", type=parse_int, default=0x02)
    parser.add_argument("--master-id-2", type=parse_int, default=0x12)
    parser.add_argument("--direction-1", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--direction-2", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--speed", type=float, default=2.5)
    parser.add_argument("--ramp-time", type=float, default=0.10)
    parser.add_argument("--acceleration", type=float, default=None)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--position-limit", type=float, default=3.0)
    parser.add_argument("--overspeed-limit", type=float, default=6.0)
    return parser


if __name__ == "__main__":
    run(argument_parser().parse_args())
