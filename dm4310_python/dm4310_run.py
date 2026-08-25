"""通过达妙官方 USB2CAN 在 Windows 上控制 DM-J4310-2EC。

默认配置来自当前设备：COM10、Slave ID 0x02、Master ID 0x12、MIT 模式。
程序以 0.5 rad/s 运行 5 秒；正常结束、异常或 Ctrl+C 都会发送失能命令。
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import serial


SERIAL_BAUD = 921_600
CONTROL_PERIOD_S = 0.010
P_MAX = 12.5
V_MAX = 30.0
T_MAX = 10.0


@dataclass
class MotorState:
    feedback_id: int
    error: int
    motor_id: int
    position: float
    velocity: float
    torque: float
    mos_temperature: int
    rotor_temperature: int


def parse_int(value: str) -> int:
    """允许输入 2、0x02、0X12 等格式。"""
    return int(value, 0)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    value = clamp(value, minimum, maximum)
    return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def pack_mit(position: float, velocity: float, kp: float, kd: float, torque: float) -> bytes:
    """把 MIT 的位置、速度、KP、KD、前馈转矩打包成 8 字节 CAN 数据。"""
    p = float_to_uint(position, -P_MAX, P_MAX, 16)
    v = float_to_uint(velocity, -V_MAX, V_MAX, 12)
    kp_u = float_to_uint(kp, 0.0, 500.0, 12)
    kd_u = float_to_uint(kd, 0.0, 5.0, 12)
    t = float_to_uint(torque, -T_MAX, T_MAX, 12)

    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0x0F) << 4) | ((t >> 8) & 0x0F),
            t & 0xFF,
        ]
    )


def usb2can_packet(can_id: int, can_data: bytes) -> bytes:
    """生成达妙官方 USB2CAN 使用的 30 字节串口发送包。"""
    if not 0 <= can_id <= 0x7FF:
        raise ValueError("标准 CAN ID 必须在 0x000..0x7FF")
    if len(can_data) != 8:
        raise ValueError("本程序要求 CAN 数据长度为 8 字节")

    packet = bytearray(
        [
            0x55,
            0xAA,
            0x1E,
            0x03,
            0x01,
            0x00,
            0x00,
            0x00,
            0x0A,
            0x00,
            0x00,
            0x00,
            0x00,
            can_id & 0xFF,
            (can_id >> 8) & 0xFF,
            0x00,
            0x00,
            0x00,
            0x08,
            0x00,
            0x00,
        ]
    )
    packet.extend(can_data)
    packet.append(0x00)
    return bytes(packet)


def send_can(port: serial.Serial, can_id: int, can_data: bytes) -> None:
    port.write(usb2can_packet(can_id, can_data))


def send_special(port: serial.Serial, slave_id: int, command: int) -> None:
    send_can(port, slave_id, bytes([0xFF] * 7 + [command]))


class FeedbackParser:
    """解析 USB2CAN 返回的 16 字节串口包。"""

    def __init__(self, master_id: int, slave_id: int):
        self.master_id = master_id
        self.slave_id = slave_id
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> list[MotorState]:
        self.buffer.extend(chunk)
        states: list[MotorState] = []

        while len(self.buffer) >= 16:
            try:
                start = self.buffer.index(0xAA)
            except ValueError:
                self.buffer.clear()
                break

            if start:
                del self.buffer[:start]
            if len(self.buffer) < 16:
                break
            if self.buffer[15] != 0x55:
                del self.buffer[0]
                continue

            packet = bytes(self.buffer[:16])
            del self.buffer[:16]

            command = packet[1]
            can_id = int.from_bytes(packet[3:7], "little")
            data = packet[7:15]
            if command != 0x11:
                continue

            motor_id = data[0] & 0x0F
            # 部分旧固件的 Master ID 仍是 0x00。只要反馈数据中的电机 ID
            # 与目标 Slave ID 一致，就接受该帧并报告实际反馈 ID。
            if motor_id != (self.slave_id & 0x0F):
                continue
            error = data[0] >> 4
            p = (data[1] << 8) | data[2]
            v = (data[3] << 4) | (data[4] >> 4)
            t = ((data[4] & 0x0F) << 8) | data[5]
            states.append(
                MotorState(
                    feedback_id=can_id,
                    error=error,
                    motor_id=motor_id,
                    position=uint_to_float(p, -P_MAX, P_MAX, 16),
                    velocity=uint_to_float(v, -V_MAX, V_MAX, 12),
                    torque=uint_to_float(t, -T_MAX, T_MAX, 12),
                    mos_temperature=data[6],
                    rotor_temperature=data[7],
                )
            )

        return states


def receive_states(port: serial.Serial, parser: FeedbackParser) -> list[MotorState]:
    waiting = port.in_waiting
    if waiting <= 0:
        return []
    return parser.feed(port.read(waiting))


def wait_for_feedback(
    port: serial.Serial, parser: FeedbackParser, timeout_s: float
) -> MotorState | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        states = receive_states(port, parser)
        if states:
            return states[-1]
        time.sleep(0.005)
    return None


def run(args: argparse.Namespace) -> None:
    if args.duration <= 0:
        raise ValueError("duration 必须大于 0；先使用有限时间测试更安全")

    port: serial.Serial | None = None
    enabled = False
    try:
        port = serial.Serial(
            port=args.port,
            baudrate=SERIAL_BAUD,
            timeout=0,
            write_timeout=0.2,
        )
        port.reset_input_buffer()
        port.reset_output_buffer()
        parser = FeedbackParser(args.master_id, args.slave_id)

        # 从已知的安全状态开始。
        send_special(port, args.slave_id, 0xFD)
        time.sleep(0.1)
        port.reset_input_buffer()

        print(
            f"连接 {args.port}，Slave=0x{args.slave_id:02X}，"
            f"Master=0x{args.master_id:02X}"
        )
        print("发送使能命令……")
        send_special(port, args.slave_id, 0xFC)
        enabled = True

        state = wait_for_feedback(port, parser, 0.8)
        if state is None:
            raise RuntimeError("使能后未收到电机反馈，已停止；检查 ID 和 CAN 波特率")
        if state.error:
            raise RuntimeError(f"电机反馈错误码 0x{state.error:X}，已停止")

        if state.feedback_id != args.master_id:
            print(
                f"提示：实际反馈 ID 是 0x{state.feedback_id:02X}，"
                f"不是命令行填写的 0x{args.master_id:02X}；本次已自动采用实际值"
            )

        print(
            f"反馈正常（ID=0x{state.feedback_id:02X}）：位置={state.position:.3f} rad，"
            f"速度={state.velocity:.3f} rad/s，开始低速运行"
        )

        command = pack_mit(
            position=0.0,
            velocity=args.speed,
            kp=0.0,
            kd=args.kd,
            torque=0.0,
        )
        print("MIT 数据：", command.hex(" ").upper())

        start = time.monotonic()
        next_send = start
        last_feedback = start
        last_print = start

        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            send_can(port, args.slave_id, command)

            states = receive_states(port, parser)
            if states:
                state = states[-1]
                last_feedback = now
                if state.error:
                    raise RuntimeError(f"运行中电机错误码 0x{state.error:X}")

            if now - last_feedback > 0.5:
                raise RuntimeError("超过 0.5 秒未收到反馈，已停止")

            if now - last_print >= 0.25:
                print(
                    f"位置={state.position:7.3f} rad  "
                    f"速度={state.velocity:6.3f} rad/s  "
                    f"转矩={state.torque:6.3f} N·m"
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
        if port is not None and port.is_open:
            try:
                if enabled:
                    # 先给零速度，再失能。
                    zero = pack_mit(0.0, 0.0, 0.0, max(args.kd, 1.0), 0.0)
                    for _ in range(5):
                        send_can(port, args.slave_id, zero)
                        time.sleep(CONTROL_PERIOD_S)
                send_special(port, args.slave_id, 0xFD)
                time.sleep(0.05)
                send_special(port, args.slave_id, 0xFD)
                print("电机已失能")
            finally:
                port.close()


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="DM-J4310 Windows USB2CAN 低速测试")
    parser.add_argument("--port", default="COM10", help="USB2CAN 串口，默认 COM10")
    parser.add_argument(
        "--slave-id", type=parse_int, default=0x02, help="电机 CAN ID，默认 0x02"
    )
    parser.add_argument(
        "--master-id", type=parse_int, default=0x12, help="反馈 ID，默认 0x12"
    )
    parser.add_argument("--speed", type=float, default=0.5, help="目标速度 rad/s")
    parser.add_argument("--kd", type=float, default=1.0, help="MIT 速度阻尼，默认 1.0")
    parser.add_argument("--duration", type=float, default=5.0, help="运行秒数，默认 5")
    return parser


if __name__ == "__main__":
    run(build_argument_parser().parse_args())
