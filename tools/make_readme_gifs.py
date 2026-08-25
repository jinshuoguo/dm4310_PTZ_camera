"""Generate compact, looping GIF previews for the GitHub README.

The source videos remain untouched. Frames are sampled uniformly across each
complete video, so a short GIF still shows the whole development demonstration.
"""

from __future__ import annotations

from pathlib import Path

import cv2
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "assets" / "gifs"
PREVIEW_SECONDS = 6
PREVIEW_FPS = 6
FRAME_COUNT = PREVIEW_SECONDS * PREVIEW_FPS
CANVAS_SIZE = (480, 270)
PALETTE_COLORS = 56

VIDEOS = {
    "达妙官方调试软件驱动电机.mp4": "01_dmtool.gif",
    "python驱动电机.mp4": "02_python_motor.gif",
    "基于python键盘控制电机.mp4": "03_keyboard_single.gif",
    "基于python键盘控制双电机.mp4": "04_keyboard_dual.gif",
    "一个电机控制另一个电机.mp4": "05_teach_follow.gif",
    "xbox手柄控制云台.mp4": "06_xbox_gamepad.gif",
    "人脸跟随.mp4": "07_face_tracking.gif",
    "手部自动跟踪.mp4": "08_hand_tracking.gif",
    "局域网设备控制云台.mp4": "09_mobile_control.gif",
}


def fit_on_canvas(frame: Image.Image) -> Image.Image:
    """Resize without distortion and place the frame on a fixed dark canvas."""
    frame = frame.convert("RGB")
    frame.thumbnail(CANVAS_SIZE, Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", CANVAS_SIZE, (12, 16, 20))
    offset = (
        (CANVAS_SIZE[0] - frame.width) // 2,
        (CANVAS_SIZE[1] - frame.height) // 2,
    )
    canvas.paste(frame, offset)
    return canvas


def build_palette(frames: list[Image.Image]) -> Image.Image:
    """Build one shared palette to reduce file size and frame-to-frame flicker."""
    thumb_size = (70, 39)
    columns = 9
    rows = (len(frames) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * thumb_size[0], rows * thumb_size[1]))
    for index, frame in enumerate(frames):
        thumb = frame.copy()
        thumb.thumbnail(thumb_size, Image.Resampling.BILINEAR)
        x = (index % columns) * thumb_size[0]
        y = (index // columns) * thumb_size[1]
        sheet.paste(thumb, (x, y))
    return sheet.quantize(colors=PALETTE_COLORS, method=Image.Quantize.MEDIANCUT)


def sample_video(source: Path) -> list[Image.Image]:
    capture = cv2.VideoCapture(str(source))
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if not capture.isOpened() or total_frames <= 0:
        capture.release()
        raise RuntimeError(f"无法读取视频：{source.name}")

    last_frame = max(total_frames - 2, 0)
    indices = [
        round(index * last_frame / max(FRAME_COUNT - 1, 1))
        for index in range(FRAME_COUNT)
    ]
    frames: list[Image.Image] = []
    for frame_index in indices:
        capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, frame = capture.read()
        if not ok:
            continue
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frames.append(fit_on_canvas(Image.fromarray(rgb)))
    capture.release()

    if len(frames) < 2:
        raise RuntimeError(f"有效帧不足：{source.name}")
    return frames


def convert(source: Path, destination: Path) -> None:
    frames = sample_video(source)
    palette = build_palette(frames)
    quantized = [
        frame.quantize(palette=palette, dither=Image.Dither.FLOYDSTEINBERG)
        for frame in frames
    ]
    destination.parent.mkdir(parents=True, exist_ok=True)
    quantized[0].save(
        destination,
        save_all=True,
        append_images=quantized[1:],
        duration=round(1000 / PREVIEW_FPS),
        loop=0,
        optimize=True,
        disposal=2,
    )


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for source_name, destination_name in VIDEOS.items():
        source = ROOT / source_name
        destination = OUTPUT_DIR / destination_name
        if not source.exists():
            raise FileNotFoundError(source)
        print(f"生成 {destination_name} ...", flush=True)
        convert(source, destination)
        print(f"  {destination.stat().st_size / 1024 / 1024:.2f} MB", flush=True)


if __name__ == "__main__":
    main()
