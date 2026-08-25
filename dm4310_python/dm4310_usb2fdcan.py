"""使用达妙 USB2FDCAN SDK 控制 DM-J4310-2EC。

当前硬件配置：经典 CAN 1 Mbps、MIT 模式、CAN ID 0x01、Master ID 0x11。
默认以 0.5 rad/s 运行 5 秒，并用 1.5 秒 S 曲线缓慢启动、缓慢停止；
正常退出、异常或 Ctrl+C 均会发送失能命令。

运行前必须完全关闭 DMTool 和 USB2CAN 图形程序，否则 SDK 无法独占 USB 设备。
请使用 Conda base 的 Python 3.13 运行，不要使用 Windows 的 ``py`` 启动器。
"""

from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass

from dmcan import (
    DmCanContext,
    dmcan_channel_can_info,
    dmcan_device_type,
    usb_rx_frame,
)


P_MAX = 12.5
V_MAX = 30.0
T_MAX = 10.0
CONTROL_PERIOD_S = 0.010


def parse_int(value: str) -> int:
    return int(value, 0)


def float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    value = max(minimum, min(maximum, value))
    return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def smoothstep(value: float) -> float:
    """把 0..1 映射为两端斜率均为零的平滑 S 曲线。"""
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def ramped_velocity(
    elapsed: float,
    duration: float,
    ramp_time: float,
    target_velocity: float,
) -> float:
    """生成包含平滑启动和停止的速度指令。"""
    if ramp_time <= 0.0:
        return target_velocity

    # 当总运行时间太短时，自动缩短两侧斜坡，使其在中点相接。
    effective_ramp = min(ramp_time, duration / 2.0)
    if elapsed < effective_ramp:
        scale = smoothstep(elapsed / effective_ramp)
    elif elapsed > duration - effective_ramp:
        scale = smoothstep((duration - elapsed) / effective_ramp)
    else:
        scale = 1.0
    return target_velocity * scale


def pack_mit(position: float, velocity: float, kp: float, kd: float, torque: float) -> bytes:
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


@dataclass
class MotorState:
    # 反馈高四位是状态码：0=失能，1=使能；其余部分值表示故障。
    status: int = 0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    mos_temperature: int = 0
    rotor_temperature: int = 0
    received_at: float = 0.0


class Feedback:
    def __init__(self, slave_id: int, master_id: int, channel: int):
        self.slave_id = slave_id
        self.master_id = master_id
        self.channel = channel
        self.state = MotorState()
        self.event = threading.Event()
        self.lock = threading.Lock()

    def callback(self, _device, frame: usb_rx_frame) -> None:
        head = frame.head
        if int(head.channel) != self.channel or int(head.can_id) != self.master_id:
            return
        if int(head.dlc) < 8:
            return

        data = [int(frame.payload[i]) for i in range(8)]
        if (data[0] & 0x0F) != (self.slave_id & 0x0F):
            return

        p = (data[1] << 8) | data[2]
        v = (data[3] << 4) | (data[4] >> 4)
        t = ((data[4] & 0x0F) << 8) | data[5]
        with self.lock:
            self.state = MotorState(
                status=(data[0] >> 4) & 0x0F,
                position=uint_to_float(p, -P_MAX, P_MAX, 16),
                velocity=uint_to_float(v, -V_MAX, V_MAX, 12),
                torque=uint_to_float(t, -T_MAX, T_MAX, 12),
                mos_temperature=data[6],
                rotor_temperature=data[7],
                received_at=time.monotonic(),
            )
        self.event.set()

    def snapshot(self) -> MotorState:
        with self.lock:
            return MotorState(**vars(self.state))


def send(device, channel: int, can_id: int, payload: bytes) -> None:
    # 当前电机和调试工具均显示 1M，使用经典 CAN，不启用 FD/BRS。
    ok = device.send_can(
        channel,
        can_id,
        len(payload),
        payload,
        canfd=False,
        ext=False,
        rtr=False,
        brs=False,
    )
    if not ok:
        raise RuntimeError(f"USB2FDCAN 发送失败：CAN ID 0x{can_id:X}")


def special(device, channel: int, slave_id: int, command: int) -> None:
    send(device, channel, slave_id, bytes([0xFF] * 7 + [command]))


def configure_classic_can_1m(device, channel: int) -> None:
    info = dmcan_channel_can_info()
    info.channel = channel
    info.canfd = False
    info.can_baudrate = 1_000_000
    info.canfd_baudrate = 1_000_000
    info.can_sp = 0.75
    info.canfd_sp = 0.75
    if not device.set_channel_baudrate(channel, info):
        raise RuntimeError("设置 USB2FDCAN 通道为经典 CAN 1 Mbps 失败")
    device.enable_channel(channel, True)


def wait_until_enabled(
    device,
    feedback: Feedback,
    channel: int,
    slave_id: int,
    timeout: float = 1.0,
) -> MotorState:
    """等待本次使能产生的 0x1 状态，忽略队列中延迟到达的失能帧。"""
    started_at = time.monotonic()
    deadline = started_at + timeout
    next_enable = started_at
    last_status = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_enable:
            special(device, channel, slave_id, 0xFC)
            next_enable = now + 0.1

        # 先清除再读取快照；如果随后到达新帧，callback 会重新置位。
        feedback.event.clear()
        state = feedback.snapshot()
        if state.received_at >= started_at:
            last_status = state.status
            if state.status == 0x1:
                return state
            if state.status not in (0x0, 0x1):
                raise RuntimeError(
                    f"使能时电机报告故障状态 0x{state.status:X}"
                )

        remaining = deadline - time.monotonic()
        if remaining > 0:
            feedback.event.wait(min(0.05, remaining))

    if last_status == 0x0:
        raise RuntimeError(
            "已收到电机反馈，但状态始终为 0x0（失能）；"
            "请先在 DMTool 中确认该电机可以使能，并确保 DMTool 已完全退出"
        )
    raise RuntimeError(
        f"使能后 {timeout:.1f} 秒内没有收到 Master ID 反馈"
    )


def run(args: argparse.Namespace) -> None:
    if args.duration <= 0:
        raise ValueError("duration 必须大于 0")
    if args.ramp_time < 0:
        raise ValueError("ramp-time 不能小于 0")
    if args.pause_time < 0:
        raise ValueError("pause-time 不能小于 0")

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

            # 从安全失能状态开始。
            special(device, args.channel, args.slave_id, 0xFD)
            # 给失能反馈留出接收时间。即便还有延迟帧，下面也会按时间和
            # 状态持续等待，不再把第一帧 0x0 误判为使能失败。
            time.sleep(0.2)
            feedback.event.clear()

            print("发送使能命令……")
            enabled = True
            state = wait_until_enabled(
                device,
                feedback,
                args.channel,
                args.slave_id,
                timeout=1.0,
            )

            print(
                f"反馈正常：位置={state.position:.3f} rad，"
                f"速度={state.velocity:.3f} rad/s"
            )
            effective_ramp = min(args.ramp_time, args.duration / 2.0)
            velocity_targets = (
                [args.speed, -args.speed] if args.bidirectional else [args.speed]
            )

            for phase_index, velocity_target in enumerate(velocity_targets):
                direction = "正转" if velocity_target >= 0 else "反转"
                target_command = pack_mit(
                    0.0, velocity_target, 0.0, args.kd, 0.0
                )
                print(
                    f"{direction}：目标={velocity_target:.3f} rad/s，"
                    f"启动/停止斜坡各 {effective_ramp:.2f} s"
                )
                print("目标 MIT 数据：", target_command.hex(" ").upper())

                start = time.monotonic()
                next_send = start
                last_print = start
                while time.monotonic() - start < args.duration:
                    elapsed = time.monotonic() - start
                    velocity_command = ramped_velocity(
                        elapsed,
                        args.duration,
                        args.ramp_time,
                        velocity_target,
                    )
                    command = pack_mit(
                        0.0, velocity_command, 0.0, args.kd, 0.0
                    )
                    send(device, args.channel, args.slave_id, command)
                    now = time.monotonic()
                    state = feedback.snapshot()

                    if state.received_at and now - state.received_at > 0.5:
                        raise RuntimeError("超过 0.5 秒未收到电机反馈")
                    if state.status != 0x1:
                        raise RuntimeError(
                            f"运行中电机退出使能状态，状态码 0x{state.status:X}"
                        )
                    if now - last_print >= 0.25:
                        print(
                            f"目标速度={velocity_command:6.3f} rad/s  "
                            f"位置={state.position:7.3f} rad  "
                            f"速度={state.velocity:6.3f} rad/s  "
                            f"转矩={state.torque:6.3f} N·m  "
                            f"温度={state.mos_temperature}/"
                            f"{state.rotor_temperature} °C  "
                            f"状态=0x{state.status:X}"
                        )
                        last_print = now

                    next_send += CONTROL_PERIOD_S
                    delay = next_send - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    else:
                        next_send = time.monotonic()

                # 每一段结束都明确发送零速；正反转之间继续发零速并停顿，
                # 避免控制报文中断，也避免直接跨过零速换向。
                zero = pack_mit(0.0, 0.0, 0.0, args.kd, 0.0)
                changing_direction = phase_index < len(velocity_targets) - 1
                pause = args.pause_time if changing_direction else 0.05
                if changing_direction:
                    print(f"已减速到零，停顿 {args.pause_time:.2f} 秒后换向")
                pause_deadline = time.monotonic() + max(pause, 0.05)
                while time.monotonic() < pause_deadline:
                    send(device, args.channel, args.slave_id, zero)
                    time.sleep(CONTROL_PERIOD_S)

        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，正在安全停止……")
        finally:
            if device is not None:
                try:
                    if enabled:
                        zero = pack_mit(0.0, 0.0, 0.0, max(args.kd, 1.0), 0.0)
                        for _ in range(5):
                            send(device, args.channel, args.slave_id, zero)
                            time.sleep(CONTROL_PERIOD_S)
                    for _ in range(3):
                        special(device, args.channel, args.slave_id, 0xFD)
                        time.sleep(0.01)
                    print("电机已失能")
                finally:
                    if channel_enabled:
                        device.enable_channel(args.channel, False)
    finally:
        # dmcan-sdk 1.0.4 在 Windows 关闭 libusb 异步传输时可能输出
        # transfer_cancelled，并偶发抛出 0xc0000008。电机失能在此之前完成，
        # 因此清理异常只记录提示，不能覆盖控制阶段的真实结果。
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
                # 防止 SDK 对同一个失效句柄在对象析构时再次清理。
                context._ctx = None


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="DM-J4310 USB2FDCAN 低速测试")
    parser.add_argument("--slave-id", type=parse_int, default=0x01)
    parser.add_argument("--master-id", type=parse_int, default=0x11)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--speed", type=float, default=0.5, help="目标速度 rad/s")
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--ramp-time",
        type=float,
        default=1.5,
        help="缓慢启动和缓慢停止各自的时间（秒）",
    )
    parser.add_argument(
        "--bidirectional",
        action="store_true",
        help="先按目标速度运行，再以相同速度反向运行",
    )
    parser.add_argument(
        "--pause-time",
        type=float,
        default=0.5,
        help="正反转换向前的零速停顿时间（秒）",
    )
    return parser


if __name__ == "__main__":
    run(argument_parser().parse_args())
