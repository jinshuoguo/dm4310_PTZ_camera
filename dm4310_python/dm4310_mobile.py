"""通过局域网手机页面控制两台 DM-J4310。

手机只向本机 HTTP 服务发送归一化摇杆量；USB2FDCAN 的打开、
使能、限位、看门狗和失能全部在单独控制线程中执行。页面断联
0.25 秒先令速度回零，1.2 秒后自动失能两台电机。
"""

from __future__ import annotations

import argparse
import hmac
import json
import math
import queue
import secrets
import socket
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlsplit

from dmcan import DmCanContext, dmcan_device_type

from dm4310_dual_keyboard import MotorRuntime, checked_state
from dm4310_face_tracker import FaceTracker
from dm4310_keyboard import clamp, unwrap_delta
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


COMMAND_STALE_S = 0.25
AUTO_DISABLE_S = 1.20
MAX_REQUEST_BYTES = 4096


@dataclass(frozen=True)
class MotionSample:
    elapsed: float
    position_1: float
    position_2: float


def tracking_error_to_velocity(
    error: float,
    deadzone: float,
    gain: float,
    speed_limit: float,
) -> float:
    if abs(error) <= deadzone:
        return 0.0
    effective = math.copysign(
        (abs(error) - deadzone) / (1.0 - deadzone),
        error,
    )
    return clamp(gain * effective, speed_limit)


def interpolate_motion(
    samples: list[MotionSample],
    elapsed: float,
    start_index: int,
) -> tuple[tuple[float, float], tuple[float, float], int, bool]:
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
    first, second = samples[index], samples[index + 1]
    duration = max(1e-6, second.elapsed - first.elapsed)
    alpha = max(0.0, min(1.0, (elapsed - first.elapsed) / duration))
    return (
        (
            first.position_1 + alpha * (second.position_1 - first.position_1),
            first.position_2 + alpha * (second.position_2 - first.position_2),
        ),
        (
            (second.position_1 - first.position_1) / duration,
            (second.position_2 - first.position_2) / duration,
        ),
        index,
        False,
    )


class DualMobileController:
    """两电机手机摇杆控制器。所有 SDK 操作只在 worker 线程中执行。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self._lock = threading.RLock()
        self._actions: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
        self._stop = threading.Event()
        self._context = None
        self._device = None
        self._channel_enabled = False
        self._motors: list[MotorRuntime] = []
        self._enabled_motors: list[MotorRuntime] = []

        self._mode = "disconnected"
        self._message = "等待连接 USB2FDCAN"
        self._enabled = False
        self._stick_x = 0.0
        self._stick_y = 0.0
        self._velocity_commands = [0.0, 0.0]
        self._last_command_at = 0.0
        self._last_control_at = time.monotonic()

        self._operation = "manual"
        self._vision_mode = "off"
        self._tracker_started = False
        self._tracker = FaceTracker(
            camera_index=args.camera_index,
            camera_name=args.camera_name,
            preview=False,
            stream=True,
            stream_fps=args.stream_fps,
            jpeg_quality=args.jpeg_quality,
        )
        self._saved_positions: tuple[float, float] | None = None
        self._return_stable_since: float | None = None
        self._recorded_motion: list[MotionSample] = []
        self._recording = False
        self._record_started_at = 0.0
        self._last_record_at = 0.0
        self._replay_elapsed = 0.0
        self._replay_last_update = 0.0
        self._replay_index = 0
        self._replay_stable_since: float | None = None

        self.max_speed = args.max_speed
        self.x_speed_scale = args.x_speed_scale
        self.ramp_time = args.ramp_time

        self._thread = threading.Thread(
            target=self._worker,
            name="dm4310-mobile-control",
            daemon=True,
        )
        self._thread.start()

    def enqueue(self, action: str, **payload: Any) -> None:
        self._actions.put((action, payload))

    def shutdown(self) -> None:
        self._stop.set()
        self._actions.put(("shutdown", {}))
        self._thread.join(timeout=4.0)

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            motor_data: list[dict[str, Any]] = []
            for motor in self._motors:
                state = motor.feedback.snapshot()
                fresh = bool(state.received_at and now - state.received_at <= 0.5)
                motor_data.append(
                    {
                        "name": motor.name,
                        "slave_id": f"0x{motor.slave_id:02X}",
                        "master_id": f"0x{motor.master_id:02X}",
                        "position": motor.relative_position,
                        "velocity": motor.direction * state.velocity if fresh else None,
                        "torque": motor.direction * state.torque if fresh else None,
                        "temperature": state.mos_temperature if fresh else None,
                        "status": state.status if state.received_at else None,
                        "feedback_fresh": fresh,
                        "command_velocity": self._velocity_commands[len(motor_data)],
                    }
                )

            command_age = (
                now - self._last_command_at if self._last_command_at else None
            )
            observation, vision_error, tracker_running = self._tracker.snapshot()
            record_duration = (
                time.monotonic() - self._record_started_at
                if self._recording
                else self._recorded_motion[-1].elapsed
                if self._recorded_motion
                else 0.0
            )
            return {
                "mode": self._mode,
                "message": self._message,
                "connected": self._device is not None,
                "enabled": self._enabled,
                "operation": self._operation,
                "vision_mode": self._vision_mode,
                "camera_running": tracker_running,
                "vision_error": vision_error,
                "target_detected": bool(
                    observation and now - observation.detected_at <= 0.5
                ),
                "saved_positions": self._saved_positions,
                "can_return": self._saved_positions is not None,
                "recording": self._recording,
                "recorded_samples": len(self._recorded_motion),
                "recorded_duration": record_duration,
                "can_replay": len(self._recorded_motion) >= 2,
                "stick": {"x": self._stick_x, "y": self._stick_y},
                "motors": motor_data,
                "max_speed": self.max_speed,
                "x_speed_scale": self.x_speed_scale,
                "ramp_time": self.ramp_time,
                "position_limit": self.args.position_limit,
                "command_age": command_age,
                "stale_after": COMMAND_STALE_S,
                "disable_after": AUTO_DISABLE_S,
                "updated_at": time.time(),
            }

    def _set_status(self, mode: str, message: str) -> None:
        with self._lock:
            self._mode = mode
            self._message = message

    def _worker(self) -> None:
        next_step = time.monotonic()
        while not self._stop.is_set():
            try:
                self._drain_actions()
                if self._enabled:
                    self._control_step(time.monotonic())
                next_step += CONTROL_PERIOD_S
                delay = next_step - time.monotonic()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_step = time.monotonic()
            except Exception as exc:
                self._fault(f"控制异常：{exc}")
                next_step = time.monotonic()
        self._close_device()

    def _drain_actions(self) -> None:
        while True:
            try:
                action, payload = self._actions.get_nowait()
            except queue.Empty:
                return

            if action == "shutdown":
                return
            if action == "connect":
                self._connect_device()
            elif action == "enable":
                self._enable_motors()
            elif action == "disable":
                self._disable_motors("已急停，两台电机均失能")
            elif action == "command":
                self._accept_command(payload)
            elif action == "config":
                self._accept_config(payload)
            elif action == "camera":
                self._toggle_camera()
            elif action == "face":
                self._toggle_tracking("face")
            elif action == "hand":
                self._toggle_tracking("hand")
            elif action == "save_position":
                self._save_position()
            elif action == "return_position":
                self._start_return()
            elif action == "record":
                self._toggle_recording()
            elif action == "replay":
                self._start_replay()
            elif action == "cancel":
                self._cancel_automatic("已取消自动动作，切回手动控制")

    def _connect_device(self) -> None:
        if self._device is not None:
            self._set_status("disabled", "USB2FDCAN 已连接，电机保持失能")
            return

        self._set_status("connecting", "正在独占连接 USB2FDCAN…")
        context = None
        device = None
        try:
            context = DmCanContext()
            count = context.find_devices(dmcan_device_type.USB2CANFD)
            if count <= 0:
                raise RuntimeError("没有找到 USB2FDCAN；请关闭 DMTool 后重新插拔")
            if self.args.device_index >= count:
                raise RuntimeError(
                    f"只发现 {count} 个设备，device-index 超出范围"
                )

            device = context.get_device(self.args.device_index)
            if not device.open():
                raise RuntimeError("设备打开失败，可能仍被 DMTool 占用")

            feedback_1 = Feedback(
                self.args.slave_id_1,
                self.args.master_id_1,
                self.args.channel,
            )
            feedback_2 = Feedback(
                self.args.slave_id_2,
                self.args.master_id_2,
                self.args.channel,
            )
            motors = [
                MotorRuntime(
                    "M1 水平",
                    self.args.slave_id_1,
                    self.args.master_id_1,
                    self.args.direction_1,
                    feedback_1,
                ),
                MotorRuntime(
                    "M2 俯仰",
                    self.args.slave_id_2,
                    self.args.master_id_2,
                    self.args.direction_2,
                    feedback_2,
                ),
            ]

            def dispatch_feedback(callback_device, frame) -> None:
                feedback_1.callback(callback_device, frame)
                feedback_2.callback(callback_device, frame)

            device.hook_recv_callback(dispatch_feedback)
            configure_classic_can_1m(device, self.args.channel)

            self._context = context
            self._device = device
            self._motors = motors
            self._channel_enabled = True
            for motor in motors:
                special(device, self.args.channel, motor.slave_id, 0xFD)
            time.sleep(0.15)
            self._enabled = False
            self._set_status(
                "disabled",
                "USB2FDCAN 已连接；将摇杆回中后长按使能",
            )
        except Exception:
            if device is not None:
                try:
                    device.close()
                except OSError:
                    pass
            if context is not None:
                try:
                    context.destroy()
                except OSError:
                    context._ctx = None
            raise

    def _enable_motors(self) -> None:
        if self._device is None or not self._motors:
            raise RuntimeError("请先连接 USB2FDCAN")
        if self._enabled:
            self._set_status("enabled", "两台电机已使能")
            return
        if abs(self._stick_x) > 0.05 or abs(self._stick_y) > 0.05:
            raise RuntimeError("使能前请先将摇杆回中")

        self._set_status("enabling", "正在使能两台电机…")
        self._enabled_motors = []
        try:
            for motor in self._motors:
                special(self._device, self.args.channel, motor.slave_id, 0xFD)
            time.sleep(0.15)

            preload = pack_mit(0.0, 0.0, 0.0, self.args.kd, 0.0)
            for _ in range(3):
                for motor in self._motors:
                    send(
                        self._device,
                        self.args.channel,
                        motor.slave_id,
                        preload,
                    )
                time.sleep(CONTROL_PERIOD_S)

            for motor in self._motors:
                state = wait_until_enabled(
                    self._device,
                    motor.feedback,
                    self.args.channel,
                    motor.slave_id,
                    timeout=1.0,
                )
                motor.previous_raw_position = state.position
                motor.relative_position = 0.0
                self._enabled_motors.append(motor)

            for motor in self._motors:
                checked_state(motor, self.args.overspeed_limit)

            now = time.monotonic()
            with self._lock:
                self._enabled = True
                self._mode = "enabled"
                self._message = "遥控就绪；松手自动制动"
                self._stick_x = 0.0
                self._stick_y = 0.0
                self._velocity_commands = [0.0, 0.0]
                self._last_command_at = now
                self._last_control_at = now
                self._operation = "manual"
                self._return_stable_since = None
                self._replay_stable_since = None
        except Exception:
            self._disable_motors("使能失败，两台电机已失能")
            raise

    def _accept_command(self, payload: dict[str, Any]) -> None:
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("摇杆数据非法")
        with self._lock:
            self._stick_x = clamp(x, 1.0)
            self._stick_y = clamp(y, 1.0)
            self._last_command_at = time.monotonic()
            if (
                math.hypot(self._stick_x, self._stick_y) > 0.02
                and self._operation
                in {"face", "hand", "return", "replay_return", "replay"}
            ):
                self._operation = "manual"
                self._vision_mode = (
                    "preview" if self._tracker_started else "off"
                )
                self._return_stable_since = None
                self._replay_stable_since = None
                self._message = "摇杆已接管，切回手动控制"

    def _accept_config(self, payload: dict[str, Any]) -> None:
        speed = float(payload.get("max_speed", self.max_speed))
        ramp_time = float(payload.get("ramp_time", self.ramp_time))
        if not 0.5 <= speed <= 5.0:
            raise ValueError("最高速度必须在 0.5..5 rad/s")
        if not 0.08 <= ramp_time <= 1.0:
            raise ValueError("响应时间必须在 0.08..1.0 秒")
        with self._lock:
            self.max_speed = speed
            self.ramp_time = ramp_time

    def _require_enabled(self) -> None:
        if not self._enabled:
            raise RuntimeError("请先连接并长按使能两台电机")

    def _start_tracker(self, mode: str) -> None:
        self._tracker.set_mode(mode)
        if not self._tracker_started:
            self._tracker.start()
            self._tracker_started = True

    def _toggle_camera(self) -> None:
        if self._tracker_started:
            if self._enabled:
                self._message = "摄像头正在回传画面"
                return
            self._tracker.stop()
            self._tracker_started = False
            self._vision_mode = "off"
            if self._operation in {"face", "hand"}:
                self._operation = "manual"
            self._message = "摄像头已关闭"
            return
        self._start_tracker("face")
        self._vision_mode = "preview"
        self._message = "摄像头已开启，画面正在回传"

    def _toggle_tracking(self, mode: str) -> None:
        self._require_enabled()
        if self._operation == mode:
            self._operation = "manual"
            self._vision_mode = "preview"
            self._message = f"{'人脸' if mode == 'face' else '手部'}跟随已关闭"
            return
        self._cancel_automatic("")
        self._start_tracker(mode)
        self._vision_mode = mode
        self._operation = mode
        self._stick_x = self._stick_y = 0.0
        self._message = (
            "人脸跟随已开启，正在等待检测人脸…"
            if mode == "face"
            else "手部跟随已开启，正在等待检测手掌…"
        )

    def _save_position(self) -> None:
        self._require_enabled()
        self._saved_positions = tuple(
            motor.relative_position for motor in self._motors
        )
        self._message = (
            f"已记录位置：M1={self._saved_positions[0]:+.3f} rad，"
            f"M2={self._saved_positions[1]:+.3f} rad"
        )

    def _start_return(self) -> None:
        self._require_enabled()
        if self._recording:
            raise RuntimeError("请先结束动作录制")
        if self._saved_positions is None:
            raise RuntimeError("还没有记录位置")
        self._cancel_automatic("")
        self._operation = "return"
        self._return_stable_since = None
        self._stick_x = self._stick_y = 0.0
        self._message = "正在返回记录位置…"

    def _toggle_recording(self) -> None:
        self._require_enabled()
        now = time.monotonic()
        if self._recording:
            self._recorded_motion.append(
                MotionSample(
                    now - self._record_started_at,
                    self._motors[0].relative_position,
                    self._motors[1].relative_position,
                )
            )
            self._recording = False
            self._message = (
                f"动作录制完成：{self._recorded_motion[-1].elapsed:.2f} 秒，"
                f"{len(self._recorded_motion)} 帧"
            )
            return
        self._cancel_automatic("")
        self._recorded_motion = [
            MotionSample(
                0.0,
                self._motors[0].relative_position,
                self._motors[1].relative_position,
            )
        ]
        self._record_started_at = now
        self._last_record_at = now
        self._recording = True
        self._message = "正在录制动作，再点一次停止"

    def _start_replay(self) -> None:
        self._require_enabled()
        if self._recording:
            raise RuntimeError("请先结束动作录制")
        if len(self._recorded_motion) < 2:
            raise RuntimeError("请先录制一段动作")
        self._cancel_automatic("")
        self._operation = "replay_return"
        self._replay_elapsed = 0.0
        self._replay_index = 0
        self._replay_stable_since = None
        self._stick_x = self._stick_y = 0.0
        self._message = "正在自动返回录制起点…"

    def _cancel_automatic(self, message: str) -> None:
        if self._operation in {"face", "hand"}:
            self._vision_mode = "preview" if self._tracker_started else "off"
        self._operation = "manual"
        self._return_stable_since = None
        self._replay_stable_since = None
        if message:
            self._message = message

    def video_frame(self) -> tuple[bytes | None, float]:
        return self._tracker.frame_snapshot()

    def _return_velocities(
        self,
        errors: list[float],
        tolerance: float,
    ) -> list[float]:
        velocities: list[float] = []
        for error in errors:
            if abs(error) <= tolerance:
                velocities.append(0.0)
            else:
                speed = min(
                    self.args.return_speed,
                    max(self.args.return_min_speed, self.args.return_gain * abs(error)),
                )
                velocities.append(math.copysign(speed, error))
        return velocities

    def _settled(self, errors, states, tolerance: float) -> bool:
        return (
            all(abs(error) <= tolerance for error in errors)
            and all(
                abs(motor.direction * state.velocity) <= 0.10
                for motor, state in zip(self._motors, states)
            )
            and all(abs(command) <= 0.10 for command in self._velocity_commands)
        )

    def _control_step(self, now: float) -> None:
        assert self._device is not None
        states = [
            checked_state(motor, self.args.overspeed_limit)
            for motor in self._motors
        ]
        for motor, state in zip(self._motors, states):
            motor_delta = unwrap_delta(state.position, motor.previous_raw_position)
            motor.previous_raw_position = state.position
            motor.relative_position += motor.direction * motor_delta

        command_age = now - self._last_command_at
        if command_age > AUTO_DISABLE_S:
            self._disable_motors("手机通信超时，两台电机已自动失能")
            return

        dt = max(0.001, min(0.03, now - self._last_control_at))
        self._last_control_at = now
        if self._recording and now - self._last_record_at >= self.args.record_interval:
            self._recorded_motion.append(
                MotionSample(
                    now - self._record_started_at,
                    self._motors[0].relative_position,
                    self._motors[1].relative_position,
                )
            )
            self._last_record_at = now
            if now - self._record_started_at >= self.args.record_max_duration:
                self._recording = False
                self._message = (
                    f"已达到最长录制时间 "
                    f"{self.args.record_max_duration:.1f} 秒"
                )

        active_accelerations = [
            max(0.5, self.max_speed * self.x_speed_scale / self.ramp_time),
            max(0.5, self.max_speed / self.ramp_time),
        ]
        if command_age > COMMAND_STALE_S:
            desired_velocities = [0.0, 0.0]
        elif self._operation in {"face", "hand"}:
            observation, vision_error, tracker_running = self._tracker.snapshot()
            tracking_name = "人脸" if self._operation == "face" else "手部"
            if vision_error:
                raise RuntimeError(f"{tracking_name}跟随错误：{vision_error}")
            if not tracker_running:
                raise RuntimeError(f"摄像头已停止，{tracking_name}跟随中断")
            timeout = (
                self.args.face_timeout
                if self._operation == "face"
                else self.args.hand_timeout
            )
            if observation is None or now - observation.detected_at > timeout:
                desired_velocities = [0.0, 0.0]
                self._message = f"未检测到{tracking_name}，电机减速停止"
            else:
                is_face = self._operation == "face"
                deadzone = self.args.face_deadzone if is_face else self.args.hand_deadzone
                gain = self.args.face_gain if is_face else self.args.hand_gain
                speed = self.args.face_speed if is_face else self.args.hand_speed
                direction_x = (
                    self.args.face_direction_x if is_face else self.args.hand_direction_x
                )
                direction_y = (
                    self.args.face_direction_y if is_face else self.args.hand_direction_y
                )
                desired_velocities = [
                    direction_x
                    * tracking_error_to_velocity(
                        observation.error_x, deadzone, gain, speed
                    ),
                    direction_y
                    * tracking_error_to_velocity(
                        observation.error_y, deadzone, gain, speed
                    ),
                ]
                acceleration = (
                    self.args.face_acceleration
                    if is_face
                    else self.args.hand_acceleration
                )
                active_accelerations = [acceleration, acceleration]
                self._message = f"{tracking_name}已锁定，自动跟随中"
        elif self._operation == "return":
            assert self._saved_positions is not None
            errors = [
                target - motor.relative_position
                for target, motor in zip(self._saved_positions, self._motors)
            ]
            desired_velocities = self._return_velocities(
                errors,
                self.args.return_tolerance,
            )
            active_accelerations = [
                self.args.replay_acceleration,
                self.args.replay_acceleration,
            ]
            if self._settled(errors, states, self.args.return_tolerance):
                if self._return_stable_since is None:
                    self._return_stable_since = now
                elif now - self._return_stable_since >= 0.30:
                    self._operation = "manual"
                    self._return_stable_since = None
                    desired_velocities = [0.0, 0.0]
                    self._message = (
                        f"已返回记录位置：M1误差={errors[0]:+.3f}，"
                        f"M2误差={errors[1]:+.3f} rad"
                    )
            else:
                self._return_stable_since = None
        elif self._operation == "replay_return":
            first = self._recorded_motion[0]
            targets = (first.position_1, first.position_2)
            errors = [
                target - motor.relative_position
                for target, motor in zip(targets, self._motors)
            ]
            desired_velocities = self._return_velocities(
                errors,
                self.args.replay_start_tolerance,
            )
            active_accelerations = [
                self.args.replay_acceleration,
                self.args.replay_acceleration,
            ]
            if self._settled(errors, states, self.args.replay_start_tolerance):
                if self._replay_stable_since is None:
                    self._replay_stable_since = now
                elif now - self._replay_stable_since >= 0.30:
                    self._operation = "replay"
                    self._replay_elapsed = 0.0
                    self._replay_last_update = now
                    self._replay_index = 0
                    self._replay_stable_since = None
                    desired_velocities = [0.0, 0.0]
                    self._message = "已回到录制起点，开始复现动作…"
            else:
                self._replay_stable_since = None
        elif self._operation == "replay":
            replay_delta = min(0.05, max(0.0, now - self._replay_last_update))
            self._replay_last_update = now
            targets, feedforward, self._replay_index, finished = interpolate_motion(
                self._recorded_motion,
                self._replay_elapsed,
                self._replay_index,
            )
            errors = [
                target - motor.relative_position
                for target, motor in zip(targets, self._motors)
            ]
            maximum_error = max(abs(error) for error in errors)
            if maximum_error > self.args.replay_hard_error_limit:
                raise RuntimeError(
                    "动作复现跟随误差过大："
                    f"M1={errors[0]:+.3f}，M2={errors[1]:+.3f} rad"
                )
            slowdown_start = self.args.replay_error_limit * 0.5
            if maximum_error <= slowdown_start:
                progress = 1.0
            elif maximum_error >= self.args.replay_error_limit:
                progress = 0.0
            else:
                progress = (
                    self.args.replay_error_limit - maximum_error
                ) / (self.args.replay_error_limit - slowdown_start)
            desired_velocities = []
            for error, feedforward_velocity in zip(errors, feedforward):
                velocity = clamp(
                    progress * feedforward_velocity + self.args.replay_gain * error,
                    self.args.replay_speed,
                )
                if (
                    abs(error) > self.args.return_tolerance
                    and abs(velocity) < self.args.return_min_speed
                ):
                    velocity = math.copysign(self.args.return_min_speed, error)
                desired_velocities.append(velocity)
            if not finished:
                self._replay_elapsed += replay_delta * progress
            active_accelerations = [
                self.args.replay_acceleration,
                self.args.replay_acceleration,
            ]
            if finished and self._settled(
                errors,
                states,
                self.args.return_tolerance,
            ):
                if self._replay_stable_since is None:
                    self._replay_stable_since = now
                elif now - self._replay_stable_since >= 0.30:
                    self._operation = "manual"
                    self._replay_stable_since = None
                    desired_velocities = [0.0, 0.0]
                    self._message = "动作复现完成"
            else:
                self._replay_stable_since = None
        else:
            desired_velocities = [
                self._stick_x * self.max_speed * self.x_speed_scale,
                self._stick_y * self.max_speed,
            ]

        for index, (motor, desired, acceleration) in enumerate(
            zip(self._motors, desired_velocities, active_accelerations)
        ):
            if desired > 0:
                remaining = self.args.position_limit - motor.relative_position
                safe_speed = math.sqrt(max(0.0, 2.0 * acceleration * remaining))
                desired = min(desired, safe_speed)
            elif desired < 0:
                remaining = self.args.position_limit + motor.relative_position
                safe_speed = math.sqrt(max(0.0, 2.0 * acceleration * remaining))
                desired = max(desired, -safe_speed)

            self._velocity_commands[index] += clamp(
                desired - self._velocity_commands[index],
                acceleration * dt,
            )

        for motor, velocity in zip(self._motors, self._velocity_commands):
            command = pack_mit(
                0.0,
                motor.direction * velocity,
                0.0,
                self.args.kd,
                0.0,
            )
            send(self._device, self.args.channel, motor.slave_id, command)

    def _disable_motors(self, message: str) -> None:
        if self._device is None:
            self._set_status("disconnected", "设备尚未连接")
            return
        motors = self._motors
        try:
            if self._enabled or self._enabled_motors:
                zero = pack_mit(0.0, 0.0, 0.0, max(self.args.kd, 1.0), 0.0)
                for _ in range(6):
                    for motor in motors:
                        try:
                            send(
                                self._device,
                                self.args.channel,
                                motor.slave_id,
                                zero,
                            )
                        except Exception:
                            pass
                    time.sleep(CONTROL_PERIOD_S)
            for _ in range(3):
                for motor in motors:
                    try:
                        special(
                            self._device,
                            self.args.channel,
                            motor.slave_id,
                            0xFD,
                        )
                    except Exception:
                        pass
                time.sleep(0.01)
        finally:
            with self._lock:
                self._enabled = False
                self._enabled_motors = []
                self._mode = "disabled"
                self._message = message
                self._stick_x = 0.0
                self._stick_y = 0.0
                self._velocity_commands = [0.0, 0.0]
                self._operation = "manual"
                self._recording = False
                self._return_stable_since = None
                self._replay_stable_since = None

    def _fault(self, message: str) -> None:
        try:
            self._disable_motors(message)
        finally:
            self._set_status("fault", message)

    def _close_device(self) -> None:
        if self._device is not None:
            self._disable_motors("服务已停止")
            if self._channel_enabled:
                try:
                    self._device.enable_channel(self.args.channel, False)
                except Exception:
                    pass
            try:
                self._device.close()
            except OSError as exc:
                print(f"提示：USB2FDCAN 关闭设备时返回 {exc}，可忽略")
        if self._context is not None:
            try:
                self._context.destroy()
            except OSError as exc:
                print(f"提示：USB2FDCAN SDK 清理句柄时返回 {exc}，可忽略")
                self._context._ctx = None
        if self._tracker_started:
            self._tracker.stop()
            self._tracker_started = False
        self._device = None
        self._context = None
        self._motors = []
        self._enabled_motors = []
        self._channel_enabled = False
        self._enabled = False


class MobileRequestHandler(BaseHTTPRequestHandler):
    controller: DualMobileController
    html_path: Path
    token: str

    def _security_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; style-src 'self' 'unsafe-inline'; "
            "script-src 'self' 'unsafe-inline'; connect-src 'self'; "
            "img-src 'self' data:; frame-ancestors 'none'",
        )

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._security_headers()
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self, query: dict[str, list[str]] | None = None) -> bool:
        supplied = self.headers.get("X-Control-Token", "")
        if not supplied and query:
            supplied = query.get("token", [""])[0]
        return bool(supplied and hmac.compare_digest(supplied, self.token))

    def do_GET(self) -> None:
        parsed = urlsplit(self.path)
        query = parse_qs(parsed.query)
        if parsed.path == "/":
            body = self.html_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self._security_headers()
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/state":
            if not self._authorized(query):
                self._json(403, {"error": "控制码无效，请使用终端中的完整地址"})
                return
            self._json(200, self.controller.snapshot())
            return
        if parsed.path == "/api/video":
            if not self._authorized(query):
                self._json(403, {"error": "控制码无效"})
                return
            self._stream_video()
            return
        self._json(404, {"error": "not found"})

    def _stream_video(self) -> None:
        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=dmframe",
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        last_frame_at = 0.0
        try:
            while True:
                frame, frame_at = self.controller.video_frame()
                if frame is None or frame_at <= last_frame_at:
                    time.sleep(0.02)
                    continue
                self.wfile.write(b"--dmframe\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                last_frame_at = frame_at
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def do_POST(self) -> None:
        parsed = urlsplit(self.path)
        query = parse_qs(parsed.query)
        if parsed.path != "/api/action":
            self._json(404, {"error": "not found"})
            return
        if not self._authorized(query):
            self._json(403, {"error": "控制码无效"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length < 0 or length > MAX_REQUEST_BYTES:
                raise ValueError("请求过大")
            raw = self.rfile.read(length) if length else b"{}"
            payload = json.loads(raw.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("请求格式错误")
            action = payload.pop("action", None)
            if action not in {
                "connect",
                "enable",
                "disable",
                "command",
                "config",
                "camera",
                "face",
                "hand",
                "save_position",
                "return_position",
                "record",
                "replay",
                "cancel",
            }:
                raise ValueError("未知操作")
            self.controller.enqueue(action, **payload)
            self._json(202, {"accepted": True, "action": action})
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._json(400, {"error": str(exc)})

    def log_message(self, _format: str, *_args: Any) -> None:
        return


def local_ipv4_addresses() -> list[str]:
    # 优先只返回拥有默认路由的地址。电脑上常有 VirtualBox、
    # Hyper-V 等无网关的 192.168.x.x 虚拟网卡，这些地址在本机
    # 测试正常，但手机无法访问。
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        primary = probe.getsockname()[0]
        probe.close()
        if primary and not primary.startswith("127."):
            return [primary]
    except OSError:
        pass

    addresses: set[str] = set()
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = item[4][0]
            if not address.startswith(("127.", "169.254.")):
                addresses.add(address)
    except OSError:
        pass
    return sorted(addresses)


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="DM4310 云台相机局域网手机遥控")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--token", default=None, help="固定连接控制码；默认每次随机生成")
    parser.add_argument("--slave-id-1", type=parse_int, default=0x01)
    parser.add_argument("--master-id-1", type=parse_int, default=0x11)
    parser.add_argument("--slave-id-2", type=parse_int, default=0x02)
    parser.add_argument("--master-id-2", type=parse_int, default=0x12)
    parser.add_argument("--direction-1", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--direction-2", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--max-speed", type=float, default=3.0)
    parser.add_argument("--x-speed-scale", type=float, default=0.60)
    parser.add_argument("--ramp-time", type=float, default=0.20)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--position-limit", type=float, default=3.0)
    parser.add_argument("--overspeed-limit", type=float, default=7.0)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-name", default="icspring")
    parser.add_argument("--stream-fps", type=float, default=15.0)
    parser.add_argument("--jpeg-quality", type=int, default=72)
    parser.add_argument("--face-speed", type=float, default=1.5)
    parser.add_argument("--face-gain", type=float, default=2.5)
    parser.add_argument("--face-deadzone", type=float, default=0.08)
    parser.add_argument("--face-timeout", type=float, default=0.5)
    parser.add_argument("--face-acceleration", type=float, default=4.0)
    parser.add_argument("--face-direction-x", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--face-direction-y", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--hand-speed", type=float, default=1.5)
    parser.add_argument("--hand-gain", type=float, default=2.5)
    parser.add_argument("--hand-deadzone", type=float, default=0.10)
    parser.add_argument("--hand-timeout", type=float, default=0.5)
    parser.add_argument("--hand-acceleration", type=float, default=4.0)
    parser.add_argument("--hand-direction-x", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--hand-direction-y", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--return-speed", type=float, default=1.5)
    parser.add_argument("--return-gain", type=float, default=4.0)
    parser.add_argument("--return-min-speed", type=float, default=0.12)
    parser.add_argument("--return-tolerance", type=float, default=0.02)
    parser.add_argument("--record-max-duration", type=float, default=30.0)
    parser.add_argument("--record-interval", type=float, default=0.02)
    parser.add_argument("--replay-speed", type=float, default=3.0)
    parser.add_argument("--replay-gain", type=float, default=6.0)
    parser.add_argument("--replay-acceleration", type=float, default=6.0)
    parser.add_argument("--replay-error-limit", type=float, default=0.6)
    parser.add_argument("--replay-hard-error-limit", type=float, default=1.2)
    parser.add_argument("--replay-start-tolerance", type=float, default=0.05)
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.slave_id_1 == args.slave_id_2:
        raise ValueError("两台电机的 Slave ID 不能相同")
    if args.master_id_1 == args.master_id_2:
        raise ValueError("两台电机的 Master ID 不能相同")
    if not 0.5 <= args.max_speed <= 5.0:
        raise ValueError("max-speed 必须在 0.5..5 rad/s")
    if not 0.1 <= args.x_speed_scale <= 1.0:
        raise ValueError("x-speed-scale 必须在 0.1..1")
    if not 0.08 <= args.ramp_time <= 1.0:
        raise ValueError("ramp-time 必须在 0.08..1.0 秒")
    if not 0.0 <= args.kd <= 5.0:
        raise ValueError("kd 必须在 0..5")
    if not 0.0 < args.position_limit < 12.5:
        raise ValueError("position-limit 必须在 0..12.5 rad")
    if args.overspeed_limit <= 5.0:
        raise ValueError("overspeed-limit 必须大于 5 rad/s")
    if not 1.0 <= args.stream_fps <= 30.0:
        raise ValueError("stream-fps 必须在 1..30")
    if not 40 <= args.jpeg_quality <= 95:
        raise ValueError("jpeg-quality 必须在 40..95")
    if not 0.0 <= args.face_deadzone < 0.4 or not 0.0 <= args.hand_deadzone < 0.4:
        raise ValueError("视觉跟随死区必须在 0..0.4")
    if not 0.0 < args.face_speed <= 2.0 or not 0.0 < args.hand_speed <= 2.0:
        raise ValueError("视觉跟随速度必须在 0..2 rad/s")
    if not 0.0 < args.face_gain <= 10.0 or not 0.0 < args.hand_gain <= 10.0:
        raise ValueError("视觉跟随增益必须在 0..10")
    if not 0.2 <= args.face_timeout <= 2.0 or not 0.2 <= args.hand_timeout <= 2.0:
        raise ValueError("视觉跟随丢失超时必须在 0.2..2 秒")
    if not 0.1 <= args.face_acceleration <= 10.0 or not 0.1 <= args.hand_acceleration <= 10.0:
        raise ValueError("视觉跟随加速度必须在 0.1..10 rad/s²")
    if not 0.0 < args.return_speed <= 5.0:
        raise ValueError("return-speed 必须在 0..5 rad/s")
    if not 0.0 <= args.return_min_speed <= args.return_speed:
        raise ValueError("return-min-speed 不能超过 return-speed")
    if not 0.0 < args.return_gain <= 20.0:
        raise ValueError("return-gain 必须在 0..20")
    if not 0.0 < args.return_tolerance <= 0.2:
        raise ValueError("return-tolerance 必须在 0..0.2 rad")
    if not 1.0 <= args.record_max_duration <= 120.0:
        raise ValueError("record-max-duration 必须在 1..120 秒")
    if not 0.01 <= args.record_interval <= 0.1:
        raise ValueError("record-interval 必须在 0.01..0.1 秒")
    if not args.replay_error_limit < args.replay_hard_error_limit <= 2.5:
        raise ValueError("动作复现硬误差阈值配置错误")
    if not 0.0 < args.replay_speed <= 5.0:
        raise ValueError("replay-speed 必须在 0..5 rad/s")
    if not 0.0 < args.replay_gain <= 20.0:
        raise ValueError("replay-gain 必须在 0..20")
    if not 0.1 <= args.replay_acceleration <= 20.0:
        raise ValueError("replay-acceleration 必须在 0.1..20 rad/s²")
    if not 0.01 <= args.replay_start_tolerance <= 0.2:
        raise ValueError("replay-start-tolerance 必须在 0.01..0.2 rad")
    if not 1 <= args.port <= 65535:
        raise ValueError("port 必须在 1..65535")


def main() -> None:
    args = argument_parser().parse_args()
    validate_args(args)
    html_path = Path(__file__).with_name("mobile_controller.html")
    if not html_path.exists():
        raise FileNotFoundError(f"缺少前端文件：{html_path}")

    token = args.token or secrets.token_urlsafe(12)
    controller = DualMobileController(args)
    MobileRequestHandler.controller = controller
    MobileRequestHandler.html_path = html_path
    MobileRequestHandler.token = token
    server = ThreadingHTTPServer((args.host, args.port), MobileRequestHandler)

    addresses = local_ipv4_addresses()
    print("\nDM-J4310 手机遥控已启动")
    if addresses:
        for address in addresses:
            print(f"  手机打开：http://{address}:{args.port}/?token={token}")
    else:
        print(f"  本机打开：http://127.0.0.1:{args.port}/?token={token}")
        print("  未检测到局域网 IPv4 地址")
    print("手机和电脑必须在同一局域网；Windows 防火墙请只允许“专用网络”。")
    print("请关闭 DMTool；终端按 Ctrl+C 会安全失能两台电机。\n")

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n正在停止服务并失能两台电机…")
    finally:
        server.shutdown()
        server.server_close()
        controller.shutdown()


if __name__ == "__main__":
    main()
