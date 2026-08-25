"""DM-J4310 本地网页实时角度控制服务。

浏览器只负责交互，所有 USB2FDCAN 操作都在本 Python 进程的控制线程中。
默认仅监听 127.0.0.1。设备连接后仍保持失能，必须先执行网页中的归零。
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from dmcan import DmCanContext, dmcan_device_type

from dm4310_usb2fdcan import (
    CONTROL_PERIOD_S,
    Feedback,
    configure_classic_can_1m,
    pack_mit,
    send,
    smoothstep,
    special,
    wait_until_enabled,
)


SLAVE_ID = 0x01
MASTER_ID = 0x11
CHANNEL = 0
ANGLE_LIMIT = 3.0


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class MotorWebController:
    def __init__(self, direction: int = -1) -> None:
        if direction not in (-1, 1):
            raise ValueError("direction 只能是 -1 或 1")
        self._lock = threading.RLock()
        self._actions: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
        self._stop = threading.Event()
        self._context = None
        self._device = None
        self._feedback: Feedback | None = None
        self._channel_enabled = False
        self._enabled = False
        # 电机原始坐标到前端/机构坐标的方向映射。当前机构实测为反向。
        self.direction = direction

        self._mode = "disconnected"
        self._message = "等待连接 USB2FDCAN"
        self._target = 0.0
        self._command_position = 0.0
        self._command_velocity = 0.0
        self._integral_torque = 0.0
        self._heartbeat_at = 0.0
        self._last_command_at = time.monotonic()

        self.max_speed = 0.40
        self.max_acceleration = 0.60
        self.kp = 2.5
        self.kd = 1.0
        self.ki = 0.60
        self.integral_limit = 0.35
        self.overspeed_limit = 2.0
        self.following_error_limit = 0.80

        self._home_started = 0.0
        self._home_stage = "approach"
        self._capture_started = 0.0
        self._settled_since: float | None = None
        self._best_home_error = 0.0
        self._last_home_progress = 0.0

        self._thread = threading.Thread(
            target=self._worker,
            name="dm4310-control",
            daemon=True,
        )
        self._thread.start()

    def enqueue(self, action: str, **payload: Any) -> None:
        self._actions.put((action, payload))

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            feedback = self._feedback.snapshot() if self._feedback else None
            return {
                "mode": self._mode,
                "message": self._message,
                "connected": self._device is not None,
                "enabled": self._enabled,
                "target": self.direction * self._target,
                "command_position": self.direction * self._command_position,
                "command_velocity": self.direction * self._command_velocity,
                "integral_torque": self.direction * self._integral_torque,
                "position": (
                    self.direction * feedback.position if feedback else None
                ),
                "velocity": (
                    self.direction * feedback.velocity if feedback else None
                ),
                "torque": self.direction * feedback.torque if feedback else None,
                "mos_temperature": feedback.mos_temperature if feedback else None,
                "rotor_temperature": feedback.rotor_temperature if feedback else None,
                "motor_status": feedback.status if feedback else None,
                "max_speed": self.max_speed,
                "max_acceleration": self.max_acceleration,
                "kp": self.kp,
                "kd": self.kd,
                "angle_limit": ANGLE_LIMIT,
                "direction": self.direction,
                "updated_at": time.time(),
            }

    def shutdown(self) -> None:
        self._stop.set()
        self._actions.put(("shutdown", {}))
        self._thread.join(timeout=3.0)

    def _set_status(self, mode: str, message: str) -> None:
        with self._lock:
            self._mode = mode
            self._message = message

    def _worker(self) -> None:
        next_step = time.monotonic()
        while not self._stop.is_set():
            try:
                self._drain_actions()
                now = time.monotonic()
                if self._mode == "homing":
                    self._step_homing(now)
                elif self._mode == "tracking":
                    self._step_tracking(now)
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
            elif action == "home":
                self._start_homing()
            elif action == "disable":
                self._disable_device("电机已由用户失能")
            elif action == "target":
                self._accept_target(payload)
            elif action == "heartbeat":
                self._heartbeat_at = time.monotonic()
            elif action == "config":
                self._accept_config(payload)

    def _connect_device(self) -> None:
        if self._device is not None:
            self._set_status("disabled", "设备已连接，电机保持失能")
            return

        self._set_status("connecting", "正在独占连接 USB2FDCAN…")
        context = DmCanContext()
        count = context.find_devices(dmcan_device_type.USB2CANFD)
        if count <= 0:
            try:
                context.destroy()
            except OSError:
                context._ctx = None
            raise RuntimeError("没有找到 USB2FDCAN；请关闭 DMTool 后重新插拔")

        device = context.get_device(0)
        if not device.open():
            try:
                context.destroy()
            except OSError:
                context._ctx = None
            raise RuntimeError("设备打开失败，可能仍被 DMTool 占用")

        feedback = Feedback(SLAVE_ID, MASTER_ID, CHANNEL)
        device.hook_recv_callback(feedback.callback)
        configure_classic_can_1m(device, CHANNEL)

        self._context = context
        self._device = device
        self._feedback = feedback
        self._channel_enabled = True
        special(device, CHANNEL, SLAVE_ID, 0xFD)
        time.sleep(0.2)
        self._enabled = False
        self._set_status("disabled", "USB2FDCAN 已连接；请先执行归零")

    def _start_homing(self) -> None:
        if self._device is None or self._feedback is None:
            raise RuntimeError("请先连接设备")
        if self._enabled:
            self._disable_device("重新准备归零")

        special(self._device, CHANNEL, SLAVE_ID, 0xFD)
        time.sleep(0.15)
        preload = pack_mit(0.0, 0.0, 0.0, self.kd, 0.0)
        for _ in range(3):
            send(self._device, CHANNEL, SLAVE_ID, preload)
            time.sleep(CONTROL_PERIOD_S)

        self._enabled = True
        state = wait_until_enabled(
            self._device,
            self._feedback,
            CHANNEL,
            SLAVE_ID,
            timeout=1.0,
        )
        if abs(state.velocity) > self.overspeed_limit:
            raise RuntimeError("使能时电机仍在高速运动")

        now = time.monotonic()
        with self._lock:
            self._mode = "homing"
            self._message = "正在安全归零，请勿触碰机构"
            self._target = 0.0
            self._command_position = state.position
            self._command_velocity = 0.0
            self._integral_torque = 0.0
        self._home_started = now
        self._home_stage = "capture" if abs(state.position) <= 0.20 else "approach"
        self._capture_started = now
        self._settled_since = None
        self._best_home_error = abs(state.position)
        self._last_home_progress = now

    def _step_homing(self, now: float) -> None:
        assert self._device is not None and self._feedback is not None
        state = self._checked_feedback()
        error = abs(state.position)
        if now - self._home_started > 30.0:
            raise RuntimeError(f"归零超时，当前位置 {state.position:.4f} rad")

        if error < self._best_home_error - 0.02:
            self._best_home_error = error
            self._last_home_progress = now

        if self._home_stage == "approach" and error <= 0.20:
            self._home_stage = "capture"
            self._capture_started = now
            self._settled_since = None
            self._integral_torque = 0.0
            self._message = "已进入零位捕获区"

        if self._home_stage == "approach":
            if now - self._last_home_progress > 3.0:
                raise RuntimeError("归零无进展，请检查机械卡滞")
            braking_distance = max(error - 0.20, 0.0)
            braking_speed = math.sqrt(2.0 * 0.30 * braking_distance)
            speed = min(0.30, max(0.12, braking_speed))
            desired_velocity = math.copysign(speed, -state.position)
            self._command_velocity += clamp(
                desired_velocity - self._command_velocity,
                0.30 * CONTROL_PERIOD_S,
            )
            command = pack_mit(0.0, self._command_velocity, 0.0, 1.0, 0.0)
        else:
            self._command_velocity = 0.0
            kp = 2.0 * smoothstep((now - self._capture_started) / 1.0)
            self._integral_torque += 1.0 * (-state.position) * CONTROL_PERIOD_S
            self._integral_torque = clamp(self._integral_torque, 0.35)
            command = pack_mit(0.0, 0.0, kp, 1.0, self._integral_torque)

            if error <= 0.005 and abs(state.velocity) <= 0.03:
                if self._settled_since is None:
                    self._settled_since = now
                elif now - self._settled_since >= 0.5:
                    with self._lock:
                        self._mode = "tracking"
                        self._message = "归零完成，可以拖动角度盘"
                        self._target = 0.0
                        self._command_position = 0.0
                        self._command_velocity = 0.0
                    self._heartbeat_at = now
                    return
            else:
                self._settled_since = None

        send(self._device, CHANNEL, SLAVE_ID, command)

    def _step_tracking(self, now: float) -> None:
        assert self._device is not None and self._feedback is not None
        if now - self._heartbeat_at > 1.0:
            self._disable_device("前端心跳超时，电机已自动失能")
            return

        state = self._checked_feedback()
        dt = max(0.001, min(0.03, now - self._last_command_at))
        self._last_command_at = now

        error = self._target - self._command_position
        if abs(error) < 1e-5:
            desired_velocity = 0.0
        else:
            stopping_speed = math.sqrt(
                max(0.0, 2.0 * self.max_acceleration * abs(error))
            )
            desired_velocity = math.copysign(
                min(self.max_speed, stopping_speed),
                error,
            )

        self._command_velocity += clamp(
            desired_velocity - self._command_velocity,
            self.max_acceleration * dt,
        )
        step = self._command_velocity * dt
        if abs(step) >= abs(error):
            self._command_position = self._target
            self._command_velocity = 0.0
        else:
            self._command_position += step

        following_error = self._command_position - state.position
        if abs(following_error) > self.following_error_limit:
            raise RuntimeError(
                f"跟随误差过大：{following_error:.3f} rad，已失能"
            )

        if abs(self._command_velocity) < 0.05:
            self._integral_torque += self.ki * following_error * dt
            self._integral_torque = clamp(
                self._integral_torque,
                self.integral_limit,
            )

        command = pack_mit(
            self._command_position,
            self._command_velocity,
            self.kp,
            self.kd,
            self._integral_torque,
        )
        send(self._device, CHANNEL, SLAVE_ID, command)

    def _checked_feedback(self):
        assert self._feedback is not None
        state = self._feedback.snapshot()
        now = time.monotonic()
        if not state.received_at or now - state.received_at > 0.5:
            raise RuntimeError("超过 0.5 秒未收到电机反馈")
        if state.status != 0x1:
            raise RuntimeError(f"电机状态异常：0x{state.status:X}")
        if abs(state.velocity) > self.overspeed_limit:
            raise RuntimeError(
                f"触发超速保护：{state.velocity:.3f} rad/s"
            )
        return state

    def _accept_target(self, payload: dict[str, Any]) -> None:
        if self._mode != "tracking":
            return
        angle = float(payload.get("angle", 0.0))
        if not math.isfinite(angle):
            return
        with self._lock:
            # 浏览器使用机构坐标；控制环内部始终保留电机原始坐标。
            self._target = self.direction * clamp(angle, ANGLE_LIMIT)
        self._heartbeat_at = time.monotonic()

    def _accept_config(self, payload: dict[str, Any]) -> None:
        speed = float(payload.get("max_speed", self.max_speed))
        acceleration = float(
            payload.get("max_acceleration", self.max_acceleration)
        )
        if not 0.05 <= speed <= 0.8:
            raise ValueError("最大速度必须在 0.05..0.8 rad/s")
        if not 0.05 <= acceleration <= 1.5:
            raise ValueError("最大加速度必须在 0.05..1.5 rad/s²")
        with self._lock:
            self.max_speed = speed
            self.max_acceleration = acceleration

    def _disable_device(self, message: str) -> None:
        if self._device is None:
            self._set_status("disconnected", "设备尚未连接")
            return
        try:
            if self._enabled:
                zero = pack_mit(0.0, 0.0, 0.0, max(self.kd, 1.0), 0.0)
                for _ in range(5):
                    try:
                        send(self._device, CHANNEL, SLAVE_ID, zero)
                    except Exception:
                        break
                    time.sleep(CONTROL_PERIOD_S)
            for _ in range(3):
                try:
                    special(self._device, CHANNEL, SLAVE_ID, 0xFD)
                except Exception:
                    break
                time.sleep(0.01)
        finally:
            self._enabled = False
            with self._lock:
                self._mode = "disabled"
                self._message = message
                self._target = 0.0
                self._command_velocity = 0.0
                self._integral_torque = 0.0

    def _fault(self, message: str) -> None:
        try:
            self._disable_device(message)
        finally:
            self._set_status("fault", message)

    def _close_device(self) -> None:
        if self._device is not None:
            self._disable_device("服务已停止")
            if self._channel_enabled:
                try:
                    self._device.enable_channel(CHANNEL, False)
                except Exception:
                    pass
            try:
                self._device.close()
            except OSError:
                pass
        if self._context is not None:
            try:
                self._context.destroy()
            except OSError:
                self._context._ctx = None
        self._device = None
        self._context = None
        self._feedback = None
        self._channel_enabled = False
        self._enabled = False


class ControlRequestHandler(BaseHTTPRequestHandler):
    controller: MotorWebController
    html_path: Path

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/":
            body = self.html_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/state":
            self._json(200, self.controller.snapshot())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                raise ValueError("请求过大")
            raw = self.rfile.read(length) if length else b"{}"
            payload = json.loads(raw.decode("utf-8"))
            action_map = {
                "/api/connect": "connect",
                "/api/home": "home",
                "/api/disable": "disable",
                "/api/target": "target",
                "/api/heartbeat": "heartbeat",
                "/api/config": "config",
            }
            action = action_map.get(self.path)
            if action is None:
                self._json(404, {"error": "not found"})
                return
            self.controller.enqueue(action, **payload)
            self._json(202, {"accepted": True, "action": action})
        except (ValueError, json.JSONDecodeError) as exc:
            self._json(400, {"error": str(exc)})

    def log_message(self, _format: str, *_args: Any) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="DM4310 云台相机网页角度控制台")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "--direction",
        type=int,
        choices=(-1, 1),
        default=-1,
        help="前端角度到电机角度的方向映射；当前机构默认 -1",
    )
    args = parser.parse_args()

    html_path = Path(__file__).with_name("motor_control.html")
    if not html_path.exists():
        raise FileNotFoundError(f"缺少前端文件：{html_path}")

    controller = MotorWebController(direction=args.direction)
    ControlRequestHandler.controller = controller
    ControlRequestHandler.html_path = html_path
    server = ThreadingHTTPServer((args.host, args.port), ControlRequestHandler)
    url = f"http://{args.host}:{args.port}/"
    print(f"DM-J4310 控制台已启动：{url}")
    print("关闭 DMTool 后，在网页中连接设备；按 Ctrl+C 停止服务并失能电机。")

    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n正在停止服务并失能电机……")
    finally:
        server.shutdown()
        server.server_close()
        controller.shutdown()


if __name__ == "__main__":
    main()
