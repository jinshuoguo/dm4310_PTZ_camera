"""使用 Xbox 左摇杆独立控制两台 DM-J4310。

左摇杆横轴控制电机1，纵轴控制电机2；偏转量线性映射为速度。
默认纵轴满摇杆速度为 5 rad/s，横轴为其 60%；速度指令在 0.25 秒内
完成加减速。主轴锁定会抑制上下推动时产生的少量左右串扰。
"""

from __future__ import annotations

import argparse
import ctypes
import math
import time
from ctypes import wintypes
from dataclasses import dataclass

from dmcan import DmCanContext, dmcan_device_type

from dm4310_dual_keyboard import MotorRuntime, checked_state
from dm4310_face_tracker import FaceTracker
from dm4310_keyboard import VK_ESCAPE, clamp, key_is_down, unwrap_delta
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


ERROR_DEVICE_NOT_CONNECTED = 1167
XINPUT_GAMEPAD_BACK = 0x0020
XINPUT_GAMEPAD_DPAD_UP = 0x0001
XINPUT_GAMEPAD_DPAD_DOWN = 0x0002
XINPUT_GAMEPAD_DPAD_LEFT = 0x0004
XINPUT_GAMEPAD_DPAD_RIGHT = 0x0008
XINPUT_GAMEPAD_A = 0x1000
XINPUT_GAMEPAD_B = 0x2000
XINPUT_GAMEPAD_X = 0x4000
XINPUT_GAMEPAD_Y = 0x8000


@dataclass(frozen=True)
class MotionSample:
    elapsed: float
    position_1: float
    position_2: float


class XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("wButtons", wintypes.WORD),
        ("bLeftTrigger", wintypes.BYTE),
        ("bRightTrigger", wintypes.BYTE),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class XInputState(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", wintypes.DWORD),
        ("Gamepad", XInputGamepad),
    ]


class XboxController:
    """最小化 XInput 封装，不依赖 pygame。"""

    def __init__(self, index: int, deadzone: float, exponent: float):
        self.index = index
        self.deadzone = deadzone
        self.exponent = exponent
        self._get_state = self._load_xinput()

    @staticmethod
    def _load_xinput():
        errors: list[str] = []
        for dll_name in (
            "xinput1_4.dll",
            "xinput1_3.dll",
            "xinput9_1_0.dll",
        ):
            try:
                dll = ctypes.WinDLL(dll_name)
                function = dll.XInputGetState
                function.argtypes = [
                    wintypes.DWORD,
                    ctypes.POINTER(XInputState),
                ]
                function.restype = wintypes.DWORD
                return function
            except (OSError, AttributeError) as exc:
                errors.append(f"{dll_name}: {exc}")
        raise RuntimeError("无法加载 Windows XInput：" + "；".join(errors))

    @staticmethod
    def _normalize_axis(value: int) -> float:
        divisor = 32767.0 if value >= 0 else 32768.0
        return value / divisor

    def read(self) -> tuple[float, float, int]:
        state = XInputState()
        result = self._get_state(self.index, ctypes.byref(state))
        if result == ERROR_DEVICE_NOT_CONNECTED:
            raise RuntimeError(f"Xbox 手柄 {self.index} 已断开")
        if result != 0:
            raise RuntimeError(f"读取 Xbox 手柄失败，XInput 错误码 {result}")

        x = self._normalize_axis(state.Gamepad.sThumbLX)
        y = self._normalize_axis(state.Gamepad.sThumbLY)
        magnitude = min(1.0, math.hypot(x, y))
        if magnitude <= self.deadzone:
            return 0.0, 0.0, state.Gamepad.wButtons

        # 圆形死区：越过死区后重新映射至完整的 0..1 行程。
        mapped_magnitude = (magnitude - self.deadzone) / (1.0 - self.deadzone)
        mapped_magnitude = mapped_magnitude ** self.exponent
        scale = mapped_magnitude / magnitude
        return x * scale, y * scale, state.Gamepad.wButtons


def suppress_cross_axis(
    x: float,
    y: float,
    lock_ratio: float,
) -> tuple[float, float]:
    """一个轴明显占优时，将另一个轴的摇杆串扰清零。"""
    if lock_ratio == 0:
        return x, y
    if abs(y) > abs(x) * lock_ratio:
        return 0.0, y
    if abs(x) > abs(y) * lock_ratio:
        return x, 0.0
    return x, y


def face_error_to_velocity(
    error: float,
    deadzone: float,
    gain: float,
    speed_limit: float,
) -> float:
    """将归一化画面偏差转换成带死区、限幅的速度。"""
    if abs(error) <= deadzone:
        return 0.0
    effective_error = math.copysign(
        (abs(error) - deadzone) / (1.0 - deadzone),
        error,
    )
    return clamp(gain * effective_error, speed_limit)


def interpolate_motion(
    samples: list[MotionSample],
    elapsed: float,
    start_index: int,
) -> tuple[tuple[float, float], tuple[float, float], int, bool]:
    """线性插值录制位置，并计算相邻采样点的前馈速度。"""
    if elapsed >= samples[-1].elapsed:
        final = samples[-1]
        return (
            (final.position_1, final.position_2),
            (0.0, 0.0),
            max(0, len(samples) - 2),
            True,
        )

    index = min(start_index, len(samples) - 2)
    while index + 1 < len(samples) and samples[index + 1].elapsed < elapsed:
        index += 1
    first = samples[index]
    second = samples[index + 1]
    duration = max(1e-6, second.elapsed - first.elapsed)
    alpha = max(0.0, min(1.0, (elapsed - first.elapsed) / duration))
    positions = (
        first.position_1 + alpha * (second.position_1 - first.position_1),
        first.position_2 + alpha * (second.position_2 - first.position_2),
    )
    velocities = (
        (second.position_1 - first.position_1) / duration,
        (second.position_2 - first.position_2) / duration,
    )
    return positions, velocities, index, False


def validate_args(args: argparse.Namespace) -> tuple[float, float]:
    if args.direction_1 not in (-1, 1) or args.direction_2 not in (-1, 1):
        raise ValueError("direction-1 和 direction-2 只能是 -1 或 1")
    if args.slave_id_1 == args.slave_id_2:
        raise ValueError("两台电机的 CAN ID 不能相同")
    if args.master_id_1 == args.master_id_2:
        raise ValueError("两台电机的 Master ID 不能相同")
    if not 0 < args.max_speed <= 5.0:
        raise ValueError("max-speed 必须在 0..5 rad/s 之间")
    if not 0.1 <= args.x_speed_scale <= 1.0:
        raise ValueError("x-speed-scale 必须在 0.1..1 之间")
    if not 0.02 <= args.ramp_time <= 2.0:
        raise ValueError("ramp-time 必须在 0.02..2 秒之间")
    if not 0.0 <= args.deadzone <= 0.5:
        raise ValueError("deadzone 必须在 0..0.5 之间")
    if not 0.5 <= args.response_exponent <= 3.0:
        raise ValueError("response-exponent 必须在 0.5..3 之间")
    if args.axis_lock_ratio != 0 and not 1.05 <= args.axis_lock_ratio <= 5.0:
        raise ValueError("axis-lock-ratio 必须为 0，或在 1.05..5 之间")
    if not 0 <= args.kd <= 5.0:
        raise ValueError("kd 必须在 0..5 之间")
    if not 0 < args.position_limit < 12.5:
        raise ValueError("position-limit 必须在 0..12.5 rad 之间")
    if args.overspeed_limit <= args.max_speed:
        raise ValueError("overspeed-limit 必须大于 max-speed")
    if not 0 < args.return_speed <= args.max_speed:
        raise ValueError("return-speed 必须在 0..max-speed 之间")
    if not 0 < args.return_gain <= 20.0:
        raise ValueError("return-gain 必须在 0..20 之间")
    if not 0 <= args.return_min_speed <= args.return_speed:
        raise ValueError("return-min-speed 必须在 0..return-speed 之间")
    if not 0.005 <= args.return_tolerance <= 0.2:
        raise ValueError("return-tolerance 必须在 0.005..0.2 rad 之间")
    if not 0 <= args.camera_index <= 10:
        raise ValueError("camera-index 必须在 0..10 之间")
    if not 0 < args.face_speed <= 2.0:
        raise ValueError("face-speed 必须在 0..2 rad/s 之间")
    if not 0 < args.face_gain <= 10.0:
        raise ValueError("face-gain 必须在 0..10 之间")
    if not 0 <= args.face_deadzone <= 0.4:
        raise ValueError("face-deadzone 必须在 0..0.4 之间")
    if not 0.2 <= args.face_timeout <= 2.0:
        raise ValueError("face-timeout 必须在 0.2..2 秒之间")
    if not 0.1 <= args.face_acceleration <= 10.0:
        raise ValueError("face-acceleration 必须在 0.1..10 rad/s² 之间")
    if args.face_direction_x not in (-1, 1) or args.face_direction_y not in (-1, 1):
        raise ValueError("face-direction-x/y 只能是 -1 或 1")
    if not 0 < args.hand_speed <= 2.0:
        raise ValueError("hand-speed 必须在 0..2 rad/s 之间")
    if not 0 < args.hand_gain <= 10.0:
        raise ValueError("hand-gain 必须在 0..10 之间")
    if not 0 <= args.hand_deadzone <= 0.4:
        raise ValueError("hand-deadzone 必须在 0..0.4 之间")
    if not 0.2 <= args.hand_timeout <= 2.0:
        raise ValueError("hand-timeout 必须在 0.2..2 秒之间")
    if not 0.1 <= args.hand_acceleration <= 10.0:
        raise ValueError("hand-acceleration 必须在 0.1..10 rad/s² 之间")
    if args.hand_direction_x not in (-1, 1) or args.hand_direction_y not in (-1, 1):
        raise ValueError("hand-direction-x/y 只能是 -1 或 1")
    if not 1.0 <= args.record_max_duration <= 120.0:
        raise ValueError("record-max-duration 必须在 1..120 秒之间")
    if not 0.01 <= args.record_interval <= 0.1:
        raise ValueError("record-interval 必须在 0.01..0.1 秒之间")
    if not 0 < args.replay_speed <= 5.0:
        raise ValueError("replay-speed 必须在 0..5 rad/s 之间")
    if not 0 < args.replay_gain <= 20.0:
        raise ValueError("replay-gain 必须在 0..20 之间")
    if not 0.1 <= args.replay_acceleration <= 20.0:
        raise ValueError("replay-acceleration 必须在 0.1..20 rad/s² 之间")
    if not 0.1 <= args.replay_error_limit <= 2.0:
        raise ValueError("replay-error-limit 必须在 0.1..2 rad 之间")
    if not args.replay_error_limit < args.replay_hard_error_limit <= 2.5:
        raise ValueError(
            "replay-hard-error-limit 必须大于 replay-error-limit，且不超过 2.5 rad"
        )
    if not 0.01 <= args.replay_start_tolerance <= 0.2:
        raise ValueError("replay-start-tolerance 必须在 0.01..0.2 rad 之间")
    if not 0 <= args.controller_index <= 3:
        raise ValueError("controller-index 必须在 0..3 之间")

    acceleration_y = (
        args.acceleration
        if args.acceleration is not None
        else args.max_speed / args.ramp_time
    )
    acceleration_x = (
        args.acceleration
        if args.acceleration is not None
        else args.max_speed * args.x_speed_scale / args.ramp_time
    )
    if not 0 < acceleration_x <= 100.0 or not 0 < acceleration_y <= 100.0:
        raise ValueError("acceleration 必须在 0..100 rad/s² 之间")
    return acceleration_x, acceleration_y


def run(args: argparse.Namespace) -> None:
    accelerations = validate_args(args)
    controller = XboxController(
        args.controller_index,
        args.deadzone,
        args.response_exponent,
    )

    # 在打开电机之前确认手柄确实存在且摇杆已回中。
    stick_x, stick_y, initial_buttons = controller.read()
    if math.hypot(stick_x, stick_y) > 0.05:
        raise RuntimeError("启动前请松开左摇杆并让它回到中心")

    face_tracker = FaceTracker(
        camera_index=args.camera_index,
        camera_name=args.camera_name,
        preview=not args.no_camera_preview,
    )

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

        for motor in motors:
            special(device, args.channel, motor.slave_id, 0xFD)
        time.sleep(0.2)

        zero_speed = pack_mit(0.0, 0.0, 0.0, args.kd, 0.0)
        for _ in range(3):
            for motor in motors:
                send(device, args.channel, motor.slave_id, zero_speed)
            time.sleep(CONTROL_PERIOD_S)

        print(
            f"Xbox 手柄 {args.controller_index} 已连接；"
            "USB2FDCAN 为经典 CAN 1 Mbps"
        )
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

        for motor in motors:
            checked_state(motor, args.overspeed_limit)

        operator_velocity_commands = [0.0, 0.0]
        saved_positions: tuple[float, float] | None = None
        returning = False
        face_tracking = False
        hand_tracking = False
        face_tracker_started = False
        return_stable_since: float | None = None
        recorded_motion: list[MotionSample] = []
        recording = False
        record_started_at = 0.0
        last_record_at = 0.0
        replaying = False
        replay_phase = ""
        replay_elapsed = 0.0
        replay_last_update = 0.0
        replay_index = 0
        replay_stable_since: float | None = None
        previous_buttons = initial_buttons
        last_print = time.monotonic()
        next_send = time.monotonic()

        print()
        print("左摇杆：横轴控制电机1，纵轴控制电机2，偏转量控制速度")
        print("X 记录两电机位置，Y 自动返回记录位置；拨动摇杆可取消回位")
        print("十字键 ↓ 开关人脸跟随；人脸丢失时自动减速停止")
        print("十字键 ↑ 开关手部跟随；手部丢失时自动减速停止")
        print("十字键 ← 开始/结束动作录制，十字键 → 复现刚录制的动作")
        print("按住 A 立即制动；按 B、Back、Esc 或 Ctrl+C 退出并失能")
        print(
            f"满摇杆速度：横轴={args.max_speed * args.x_speed_scale:.2f}，"
            f"纵轴={args.max_speed:.2f} rad/s；"
            f"响应={args.ramp_time:.3f} s，死区={args.deadzone:.2f}，"
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

            stick_x, stick_y, buttons = controller.read()
            stick_x, stick_y = suppress_cross_axis(
                stick_x,
                stick_y,
                args.axis_lock_ratio,
            )
            pressed_buttons = buttons & ~previous_buttons
            previous_buttons = buttons

            exit_pressed = bool(
                buttons & (XINPUT_GAMEPAD_B | XINPUT_GAMEPAD_BACK)
            ) or key_is_down(VK_ESCAPE)
            if exit_pressed:
                print("\n收到退出命令，正在停止并失能两台电机……")
                break

            brake_pressed = bool(buttons & XINPUT_GAMEPAD_A)

            if pressed_buttons & XINPUT_GAMEPAD_DPAD_LEFT:
                if recording:
                    elapsed = time.monotonic() - record_started_at
                    recorded_motion.append(
                        MotionSample(
                            elapsed,
                            motors[0].relative_position,
                            motors[1].relative_position,
                        )
                    )
                    recording = False
                    print(
                        f"\n动作录制结束：{elapsed:.2f} 秒，"
                        f"{len(recorded_motion)} 个采样点"
                    )
                else:
                    returning = False
                    face_tracking = False
                    hand_tracking = False
                    replaying = False
                    return_stable_since = None
                    replay_stable_since = None
                    recorded_motion = [
                        MotionSample(
                            0.0,
                            motors[0].relative_position,
                            motors[1].relative_position,
                        )
                    ]
                    record_started_at = time.monotonic()
                    last_record_at = record_started_at
                    recording = True
                    print("\n动作录制开始，请使用左摇杆操作两台电机……")

            if pressed_buttons & XINPUT_GAMEPAD_DPAD_RIGHT:
                if recording:
                    print("\n正在录制，请先按十字键 ← 结束录制")
                elif len(recorded_motion) < 2:
                    print("\n没有可复现的动作，请先按十字键 ← 录制")
                else:
                    returning = False
                    face_tracking = False
                    hand_tracking = False
                    replaying = True
                    replay_phase = "return_start"
                    replay_stable_since = None
                    replay_index = 0
                    print(
                        f"\n准备复现 {recorded_motion[-1].elapsed:.2f} 秒动作，"
                        "先返回录制起点……"
                    )

            if pressed_buttons & XINPUT_GAMEPAD_X:
                saved_positions = (
                    motors[0].relative_position,
                    motors[1].relative_position,
                )
                returning = False
                replaying = False
                return_stable_since = None
                print(
                    f"\n已记录位置：M1={saved_positions[0]:+.4f} rad，"
                    f"M2={saved_positions[1]:+.4f} rad"
                )

            if pressed_buttons & XINPUT_GAMEPAD_Y:
                if saved_positions is None:
                    print("\n尚未记录位置，请先按 X")
                else:
                    face_tracking = False
                    hand_tracking = False
                    replaying = False
                    returning = True
                    return_stable_since = None
                    print(
                        f"\n开始返回记录位置：M1={saved_positions[0]:+.4f} rad，"
                        f"M2={saved_positions[1]:+.4f} rad"
                    )

            if pressed_buttons & XINPUT_GAMEPAD_DPAD_DOWN:
                returning = False
                replaying = False
                return_stable_since = None
                if face_tracking:
                    face_tracking = False
                    print("\n人脸跟随已关闭")
                else:
                    hand_tracking = False
                    face_tracker.set_mode("face")
                    if not face_tracker_started:
                        face_tracker.start()
                        face_tracker_started = True
                    face_tracking = True
                    print("\n人脸跟随已开启，正在等待摄像头检测人脸……")

            if pressed_buttons & XINPUT_GAMEPAD_DPAD_UP:
                returning = False
                replaying = False
                return_stable_since = None
                if hand_tracking:
                    hand_tracking = False
                    print("\n手部跟随已关闭")
                else:
                    face_tracking = False
                    face_tracker.set_mode("hand")
                    if not face_tracker_started:
                        face_tracker.start()
                        face_tracker_started = True
                    hand_tracking = True
                    print("\n手部跟随已开启，正在等待摄像头检测手掌……")

            # 自动回位期间只要主动拨动摇杆，就立即交还人工控制。
            manual_override = math.hypot(stick_x, stick_y) > 0.02
            if returning and (brake_pressed or manual_override):
                returning = False
                return_stable_since = None
                print("\n自动回位已取消")
            if face_tracking and (brake_pressed or manual_override):
                face_tracking = False
                print("\n人脸跟随已取消，切换到手动控制")
            if hand_tracking and (brake_pressed or manual_override):
                hand_tracking = False
                print("\n手部跟随已取消，切换到手动控制")
            if replaying and (brake_pressed or manual_override):
                replaying = False
                replay_stable_since = None
                print("\n动作复现已取消，切换到手动控制")

            if recording:
                now = time.monotonic()
                elapsed = now - record_started_at
                if now - last_record_at >= args.record_interval:
                    recorded_motion.append(
                        MotionSample(
                            elapsed,
                            motors[0].relative_position,
                            motors[1].relative_position,
                        )
                    )
                    last_record_at = now
                if elapsed >= args.record_max_duration:
                    recording = False
                    print(
                        f"\n已达到最长录制时间 {args.record_max_duration:.1f} 秒，"
                        "录制自动结束"
                    )

            if brake_pressed:
                desired_velocities = [0.0, 0.0]
                operator_velocity_commands = [0.0, 0.0]
            elif replaying:
                now = time.monotonic()
                if replay_phase == "return_start":
                    first_sample = recorded_motion[0]
                    replay_targets = (
                        first_sample.position_1,
                        first_sample.position_2,
                    )
                    replay_errors = [
                        replay_targets[index] - motor.relative_position
                        for index, motor in enumerate(motors)
                    ]
                    desired_velocities = []
                    for error in replay_errors:
                        if abs(error) <= args.replay_start_tolerance:
                            desired_velocities.append(0.0)
                        else:
                            speed = min(
                                args.return_speed,
                                max(
                                    args.return_min_speed,
                                    args.return_gain * abs(error),
                                ),
                            )
                            desired_velocities.append(
                                math.copysign(speed, error)
                            )

                    settled = all(
                        abs(error) <= args.replay_start_tolerance
                        for error in replay_errors
                    ) and all(
                        abs(motor.direction * state.velocity) <= 0.10
                        for motor, state in zip(motors, states)
                    ) and all(
                        abs(command) <= 0.10
                        for command in operator_velocity_commands
                    )
                    if settled:
                        if replay_stable_since is None:
                            replay_stable_since = now
                        elif now - replay_stable_since >= 0.30:
                            replay_phase = "playback"
                            replay_elapsed = 0.0
                            replay_last_update = now
                            replay_index = 0
                            replay_stable_since = None
                            desired_velocities = [0.0, 0.0]
                            print("\n已到录制起点，开始复现动作……")
                    else:
                        replay_stable_since = None
                elif replay_phase == "playback":
                    replay_delta_time = min(
                        0.05,
                        max(0.0, now - replay_last_update),
                    )
                    replay_last_update = now
                    (
                        replay_targets,
                        feedforward_velocities,
                        replay_index,
                        playback_finished,
                    ) = interpolate_motion(
                        recorded_motion,
                        replay_elapsed,
                        replay_index,
                    )
                    replay_errors = [
                        replay_targets[index] - motor.relative_position
                        for index, motor in enumerate(motors)
                    ]
                    maximum_replay_error = max(
                        abs(error) for error in replay_errors
                    )
                    if maximum_replay_error > args.replay_hard_error_limit:
                        raise RuntimeError(
                            "动作复现跟随误差超过硬保护阈值："
                            f"M1={replay_errors[0]:+.3f} rad，"
                            f"M2={replay_errors[1]:+.3f} rad"
                        )

                    # 跟随误差达到软阈值时冻结轨迹时间，待电机追上后继续；
                    # 在半个阈值到完整阈值之间线性减慢播放速度。
                    slowdown_start = args.replay_error_limit * 0.5
                    if maximum_replay_error <= slowdown_start:
                        replay_progress = 1.0
                    elif maximum_replay_error >= args.replay_error_limit:
                        replay_progress = 0.0
                    else:
                        replay_progress = (
                            args.replay_error_limit - maximum_replay_error
                        ) / (args.replay_error_limit - slowdown_start)

                    desired_velocities = []
                    for error, feedforward in zip(
                        replay_errors,
                        feedforward_velocities,
                    ):
                        velocity = clamp(
                            replay_progress * feedforward
                            + args.replay_gain * error,
                            args.replay_speed,
                        )
                        if (
                            abs(error) > args.return_tolerance
                            and abs(velocity) < args.return_min_speed
                        ):
                            velocity = math.copysign(
                                args.return_min_speed,
                                error,
                            )
                        desired_velocities.append(velocity)

                    if not playback_finished:
                        replay_elapsed += (
                            replay_delta_time * replay_progress
                        )

                    settled = playback_finished and all(
                        abs(error) <= args.return_tolerance
                        for error in replay_errors
                    ) and all(
                        abs(motor.direction * state.velocity) <= 0.10
                        for motor, state in zip(motors, states)
                    ) and all(
                        abs(command) <= 0.10
                        for command in operator_velocity_commands
                    )
                    if settled:
                        if replay_stable_since is None:
                            replay_stable_since = now
                        elif now - replay_stable_since >= 0.30:
                            replaying = False
                            replay_phase = ""
                            replay_stable_since = None
                            desired_velocities = [0.0, 0.0]
                            print(
                                f"\n动作复现完成："
                                f"M1误差={replay_errors[0]:+.4f} rad，"
                                f"M2误差={replay_errors[1]:+.4f} rad"
                            )
                    else:
                        replay_stable_since = None
                else:
                    raise RuntimeError(f"未知动作复现阶段：{replay_phase}")
            elif returning and saved_positions is not None:
                return_errors = [
                    saved_positions[index] - motor.relative_position
                    for index, motor in enumerate(motors)
                ]
                desired_velocities = []
                for error in return_errors:
                    if abs(error) <= args.return_tolerance:
                        desired_velocities.append(0.0)
                    else:
                        return_velocity = min(
                            args.return_speed,
                            max(
                                args.return_min_speed,
                                args.return_gain * abs(error),
                            ),
                        )
                        desired_velocities.append(
                            math.copysign(return_velocity, error)
                        )

                now = time.monotonic()
                settled = all(
                    abs(error) <= args.return_tolerance
                    for error in return_errors
                ) and all(
                    abs(motor.direction * state.velocity) <= 0.10
                    for motor, state in zip(motors, states)
                ) and all(
                    abs(command) <= 0.10
                    for command in operator_velocity_commands
                )
                if settled:
                    if return_stable_since is None:
                        return_stable_since = now
                    elif now - return_stable_since >= 0.30:
                        returning = False
                        return_stable_since = None
                        desired_velocities = [0.0, 0.0]
                        print(
                            f"\n已返回记录位置："
                            f"M1误差={return_errors[0]:+.4f} rad，"
                            f"M2误差={return_errors[1]:+.4f} rad"
                        )
                else:
                    return_stable_since = None
            elif face_tracking or hand_tracking:
                observation, vision_error, tracker_running = face_tracker.snapshot()
                tracking_name = "人脸" if face_tracking else "手部"
                if vision_error is not None:
                    raise RuntimeError(f"{tracking_name}跟随错误：{vision_error}")
                if face_tracker_started and not tracker_running:
                    face_tracking = False
                    hand_tracking = False
                    face_tracker_started = False
                    desired_velocities = [0.0, 0.0]
                    print(f"\n摄像头预览已关闭，{tracking_name}跟随停止")
                elif (
                    observation is None
                    or time.monotonic() - observation.detected_at
                    > (args.face_timeout if face_tracking else args.hand_timeout)
                ):
                    desired_velocities = [0.0, 0.0]
                else:
                    tracking_deadzone = (
                        args.face_deadzone
                        if face_tracking
                        else args.hand_deadzone
                    )
                    tracking_gain = (
                        args.face_gain if face_tracking else args.hand_gain
                    )
                    tracking_speed = (
                        args.face_speed if face_tracking else args.hand_speed
                    )
                    tracking_direction_x = (
                        args.face_direction_x
                        if face_tracking
                        else args.hand_direction_x
                    )
                    tracking_direction_y = (
                        args.face_direction_y
                        if face_tracking
                        else args.hand_direction_y
                    )
                    desired_velocities = [
                        tracking_direction_x
                        * face_error_to_velocity(
                            observation.error_x,
                            tracking_deadzone,
                            tracking_gain,
                            tracking_speed,
                        ),
                        tracking_direction_y
                        * face_error_to_velocity(
                            observation.error_y,
                            tracking_deadzone,
                            tracking_gain,
                            tracking_speed,
                        ),
                    ]
            else:
                desired_velocities = [
                    args.max_speed * args.x_speed_scale * stick_x,
                    args.max_speed * stick_y,
                ]

            active_accelerations = (
                (args.face_acceleration, args.face_acceleration)
                if face_tracking
                else (args.hand_acceleration, args.hand_acceleration)
                if hand_tracking
                else (args.replay_acceleration, args.replay_acceleration)
                if replaying
                else accelerations
            )
            for index, (motor, desired_velocity, acceleration) in enumerate(
                zip(motors, desired_velocities, active_accelerations)
            ):
                # 根据剩余距离限制软限位附近的安全制动速度。
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
                send(device, args.channel, motor.slave_id, command)

            now = time.monotonic()
            if now - last_print >= 0.10:
                mode = (
                    "制动"
                    if brake_pressed
                    else "复现"
                    if replaying
                    else "回位"
                    if returning
                    else "人脸"
                    if face_tracking
                    else "手部"
                    if hand_tracking
                    else "录制"
                    if recording
                    else "运行"
                )
                print(
                    f"\r[{mode}] 摇杆 X={stick_x:+5.2f} Y={stick_y:+5.2f}  "
                    f"P1={motors[0].relative_position:+6.2f} "
                    f"C1={operator_velocity_commands[0]:+5.2f}  "
                    f"P2={motors[1].relative_position:+6.2f} "
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
        print("\n收到 Ctrl+C，正在停止并失能两台电机……")
    finally:
        if device is not None:
            stop_command = pack_mit(
                0.0,
                0.0,
                0.0,
                max(args.kd, 1.0),
                0.0,
            )
            for _ in range(8):
                for motor in enabled_motors:
                    try:
                        send(device, args.channel, motor.slave_id, stop_command)
                    except Exception:
                        pass
                time.sleep(CONTROL_PERIOD_S)

            for _ in range(3):
                for motor in enabled_motors:
                    try:
                        special(device, args.channel, motor.slave_id, 0xFD)
                    except Exception:
                        pass
                time.sleep(0.01)
            if enabled_motors:
                print("\n两台电机已失能")

            face_tracker.stop()

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
        description="Xbox 左摇杆控制 DM4310 云台相机"
    )
    parser.add_argument("--slave-id-1", type=parse_int, default=0x01)
    parser.add_argument("--master-id-1", type=parse_int, default=0x11)
    parser.add_argument("--slave-id-2", type=parse_int, default=0x02)
    parser.add_argument("--master-id-2", type=parse_int, default=0x12)
    parser.add_argument("--direction-1", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--direction-2", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--controller-index", type=int, default=0)
    parser.add_argument("--max-speed", type=float, default=5.0)
    parser.add_argument(
        "--x-speed-scale",
        type=float,
        default=0.60,
        help="横轴最高速度相对 max-speed 的比例",
    )
    parser.add_argument("--ramp-time", type=float, default=0.25)
    parser.add_argument("--acceleration", type=float, default=None)
    parser.add_argument("--deadzone", type=float, default=0.10)
    parser.add_argument("--response-exponent", type=float, default=1.0)
    parser.add_argument(
        "--axis-lock-ratio",
        type=float,
        default=1.5,
        help="主轴/次轴幅度达到该比例时抑制次轴；0 表示关闭",
    )
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--position-limit", type=float, default=3.0)
    parser.add_argument("--overspeed-limit", type=float, default=6.0)
    parser.add_argument("--return-speed", type=float, default=1.5)
    parser.add_argument("--return-gain", type=float, default=4.0)
    parser.add_argument("--return-min-speed", type=float, default=0.12)
    parser.add_argument("--return-tolerance", type=float, default=0.02)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument(
        "--camera-name",
        type=str,
        default="icspring",
        help="按设备名称选择摄像头；传空字符串时改用 camera-index",
    )
    parser.add_argument("--face-speed", type=float, default=1.5)
    parser.add_argument("--face-gain", type=float, default=2.5)
    parser.add_argument("--face-deadzone", type=float, default=0.08)
    parser.add_argument("--face-timeout", type=float, default=0.5)
    parser.add_argument("--face-acceleration", type=float, default=4.0)
    parser.add_argument(
        "--face-direction-x",
        type=int,
        choices=(-1, 1),
        default=-1,
    )
    parser.add_argument(
        "--face-direction-y",
        type=int,
        choices=(-1, 1),
        default=1,
    )
    parser.add_argument("--hand-speed", type=float, default=1.5)
    parser.add_argument("--hand-gain", type=float, default=2.5)
    parser.add_argument("--hand-deadzone", type=float, default=0.10)
    parser.add_argument("--hand-timeout", type=float, default=0.5)
    parser.add_argument("--hand-acceleration", type=float, default=4.0)
    parser.add_argument(
        "--hand-direction-x",
        type=int,
        choices=(-1, 1),
        default=-1,
    )
    parser.add_argument(
        "--hand-direction-y",
        type=int,
        choices=(-1, 1),
        default=1,
    )
    parser.add_argument("--no-camera-preview", action="store_true")
    parser.add_argument("--record-max-duration", type=float, default=30.0)
    parser.add_argument("--record-interval", type=float, default=0.02)
    parser.add_argument("--replay-speed", type=float, default=3.0)
    parser.add_argument("--replay-gain", type=float, default=6.0)
    parser.add_argument("--replay-acceleration", type=float, default=6.0)
    parser.add_argument("--replay-error-limit", type=float, default=0.6)
    parser.add_argument("--replay-hard-error-limit", type=float, default=1.2)
    parser.add_argument("--replay-start-tolerance", type=float, default=0.05)
    return parser


if __name__ == "__main__":
    run(argument_parser().parse_args())
