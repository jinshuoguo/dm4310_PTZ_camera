"""共用摄像头的人脸与手部中心检测线程。"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class FaceObservation:
    error_x: float
    error_y: float
    detected_at: float


class FaceTracker:
    """后台采集摄像头并输出归一化的人脸或手部中心偏差。"""

    def __init__(
        self,
        camera_index: int = 0,
        camera_name: str = "icspring",
        width: int = 640,
        height: int = 480,
        preview: bool = True,
        stream: bool = False,
        stream_fps: float = 15.0,
        jpeg_quality: int = 72,
        smoothing: float = 0.35,
        hand_model_path: str | None = None,
        face_model_path: str | None = None,
    ) -> None:
        self.camera_index = camera_index
        self.camera_name = camera_name.strip()
        self.width = width
        self.height = height
        self.preview = preview
        self.stream = stream
        self.stream_fps = max(1.0, min(30.0, stream_fps))
        self.jpeg_quality = max(40, min(95, jpeg_quality))
        self.smoothing = smoothing
        self.hand_model_path = Path(
            hand_model_path
            or Path(__file__).with_name("models") / "hand_landmarker.task"
        )
        self.face_model_path = Path(
            face_model_path
            or Path(__file__).with_name("models")
            / "blaze_face_short_range.tflite"
        )
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._observation: FaceObservation | None = None
        self._error: str | None = None
        self._running = False
        self._mode = "face"
        self._jpeg_frame: bytes | None = None
        self._frame_at = 0.0

    def set_mode(self, mode: str) -> None:
        if mode not in ("face", "hand"):
            raise ValueError("视觉跟随模式只能是 face 或 hand")
        with self._lock:
            if self._mode != mode:
                self._mode = mode
                self._observation = None
                self._error = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        with self._lock:
            self._observation = None
            self._error = None
            self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="face-tracker",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def snapshot(self) -> tuple[FaceObservation | None, str | None, bool]:
        with self._lock:
            return self._observation, self._error, self._running

    def frame_snapshot(self) -> tuple[bytes | None, float]:
        """返回最新的已标注 JPEG 帧，供局域网 MJPEG 流读取。"""
        with self._lock:
            return self._jpeg_frame, self._frame_at

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._error = message

    def _run(self) -> None:
        capture = None
        cv2 = None
        face_detector = None
        hand_landmarker = None
        mediapipe = None
        pending_frame = None
        window_name = "DM4310 Vision Tracking"
        try:
            try:
                import cv2 as imported_cv2
            except ImportError as exc:
                raise RuntimeError(
                    "缺少 OpenCV，请运行：python -m pip install opencv-python"
                ) from exc
            cv2 = imported_cv2

            candidates: list[tuple[int, int, str]] = []
            if self.camera_name:
                try:
                    from cv2_enumerate_cameras import enumerate_cameras
                except ImportError as exc:
                    raise RuntimeError(
                        "缺少摄像头枚举组件，请运行："
                        "python -m pip install cv2-enumerate-cameras"
                    ) from exc

                available_names: list[str] = []
                # icspring 在当前电脑上使用 MSMF 更容易取得图像，故优先。
                for backend in (cv2.CAP_MSMF, cv2.CAP_DSHOW):
                    for camera in enumerate_cameras(backend):
                        available_names.append(camera.name)
                        if self.camera_name.casefold() in camera.name.casefold():
                            candidates.append(
                                (camera.index, backend, camera.name)
                            )
                if not candidates:
                    names = "、".join(dict.fromkeys(available_names)) or "无"
                    raise RuntimeError(
                        f"未找到名称包含“{self.camera_name}”的摄像头；"
                        f"已发现：{names}"
                    )
            else:
                candidates = [
                    (self.camera_index, cv2.CAP_DSHOW, f"索引 {self.camera_index}"),
                    (self.camera_index, cv2.CAP_MSMF, f"索引 {self.camera_index}"),
                ]

            open_errors: list[str] = []
            selected_name = ""
            for index, backend, name in candidates:
                candidate = cv2.VideoCapture(index, backend)
                if not candidate.isOpened():
                    open_errors.append(f"{name}/{backend} 无法打开")
                    candidate.release()
                    continue
                candidate.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                candidate.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                candidate.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                for _ in range(8):
                    ok, frame = candidate.read()
                    if ok and frame is not None:
                        capture = candidate
                        pending_frame = frame
                        selected_name = name
                        break
                    time.sleep(0.05)
                if capture is not None:
                    break
                open_errors.append(f"{name}/{backend} 打开后无法读取图像")
                candidate.release()

            if capture is None:
                raise RuntimeError(
                    "目标摄像头无法出图，请重新插拔摄像头后重试："
                    + "；".join(open_errors)
                )
            print(f"\n摄像头已连接：{selected_name}")

            filtered_x: float | None = None
            filtered_y: float | None = None
            current_mode = ""
            last_face_timestamp_ms = 0
            last_hand_timestamp_ms = 0
            last_stream_at = 0.0
            failed_reads = 0

            while not self._stop_event.is_set():
                if pending_frame is not None:
                    ok, frame = True, pending_frame
                    pending_frame = None
                else:
                    ok, frame = capture.read()
                if not ok:
                    failed_reads += 1
                    if failed_reads >= 10:
                        raise RuntimeError("摄像头连续 10 帧读取失败")
                    time.sleep(0.02)
                    continue
                failed_reads = 0

                frame_height, frame_width = frame.shape[:2]
                frame_center_x = frame_width / 2.0
                frame_center_y = frame_height / 2.0
                selected_face = None
                face_confidence = 0.0
                hand_points: list[tuple[int, int]] = []
                target_center: tuple[float, float] | None = None
                with self._lock:
                    mode = self._mode
                if mode != current_mode:
                    current_mode = mode
                    filtered_x = filtered_y = None
                    with self._lock:
                        self._observation = None

                if mode == "face":
                    if face_detector is None:
                        if not self.face_model_path.is_file():
                            raise RuntimeError(
                                f"找不到 BlazeFace 模型：{self.face_model_path}"
                            )
                        if mediapipe is None:
                            try:
                                import mediapipe as imported_mediapipe
                            except ImportError as exc:
                                raise RuntimeError(
                                    "缺少 MediaPipe，请运行："
                                    "python -m pip install mediapipe"
                                ) from exc
                            mediapipe = imported_mediapipe
                        options = mediapipe.tasks.vision.FaceDetectorOptions(
                            base_options=mediapipe.tasks.BaseOptions(
                                model_asset_path=str(self.face_model_path)
                            ),
                            running_mode=mediapipe.tasks.vision.RunningMode.VIDEO,
                            min_detection_confidence=0.55,
                            min_suppression_threshold=0.3,
                        )
                        face_detector = (
                            mediapipe.tasks.vision.FaceDetector.create_from_options(
                                options
                            )
                        )

                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    media_image = mediapipe.Image(
                        image_format=mediapipe.ImageFormat.SRGB,
                        data=rgb_frame,
                    )
                    timestamp_ms = max(
                        last_face_timestamp_ms + 1,
                        int(time.monotonic() * 1000),
                    )
                    last_face_timestamp_ms = timestamp_ms
                    result = face_detector.detect_for_video(
                        media_image,
                        timestamp_ms,
                    )
                    if result.detections:
                        # 多人时跟随面积最大、通常也最靠近摄像头的人脸。
                        detection = max(
                            result.detections,
                            key=lambda item: (
                                item.bounding_box.width
                                * item.bounding_box.height
                            ),
                        )
                        box = detection.bounding_box
                        selected_face = (
                            box.origin_x,
                            box.origin_y,
                            box.width,
                            box.height,
                        )
                        if detection.categories:
                            face_confidence = detection.categories[0].score

                        # BlazeFace 的前 3 个关键点依次为双眼与鼻尖；使用
                        # 三点中心比边界框中心更不容易随头部姿态抖动。
                        keypoints = detection.keypoints
                        if len(keypoints) >= 3:
                            target_center = (
                                sum(point.x for point in keypoints[:3])
                                / 3.0
                                * frame_width,
                                sum(point.y for point in keypoints[:3])
                                / 3.0
                                * frame_height,
                            )
                        else:
                            target_center = (
                                box.origin_x + box.width / 2.0,
                                box.origin_y + box.height / 2.0,
                            )
                else:
                    if hand_landmarker is None:
                        if not self.hand_model_path.is_file():
                            raise RuntimeError(
                                f"找不到手部模型：{self.hand_model_path}"
                            )
                        if mediapipe is None:
                            try:
                                import mediapipe as imported_mediapipe
                            except ImportError as exc:
                                raise RuntimeError(
                                    "缺少 MediaPipe，请运行："
                                    "python -m pip install mediapipe"
                                ) from exc
                            mediapipe = imported_mediapipe
                        options = mediapipe.tasks.vision.HandLandmarkerOptions(
                            base_options=mediapipe.tasks.BaseOptions(
                                model_asset_path=str(self.hand_model_path)
                            ),
                            running_mode=mediapipe.tasks.vision.RunningMode.VIDEO,
                            num_hands=1,
                            min_hand_detection_confidence=0.5,
                            min_hand_presence_confidence=0.5,
                            min_tracking_confidence=0.5,
                        )
                        hand_landmarker = (
                            mediapipe.tasks.vision.HandLandmarker.create_from_options(
                                options
                            )
                        )

                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    media_image = mediapipe.Image(
                        image_format=mediapipe.ImageFormat.SRGB,
                        data=rgb_frame,
                    )
                    timestamp_ms = max(
                        last_hand_timestamp_ms + 1,
                        int(time.monotonic() * 1000),
                    )
                    last_hand_timestamp_ms = timestamp_ms
                    result = hand_landmarker.detect_for_video(
                        media_image,
                        timestamp_ms,
                    )
                    if result.hand_landmarks:
                        landmarks = result.hand_landmarks[0]
                        hand_points = [
                            (
                                int(landmark.x * frame_width),
                                int(landmark.y * frame_height),
                            )
                            for landmark in landmarks
                        ]
                        # 用掌根及四个指根的中心表示手掌，减少指尖动作抖动。
                        palm_indices = (0, 5, 9, 13, 17)
                        target_center = (
                            sum(hand_points[index][0] for index in palm_indices)
                            / len(palm_indices),
                            sum(hand_points[index][1] for index in palm_indices)
                            / len(palm_indices),
                        )

                if target_center is not None:
                    target_x, target_y = target_center
                    raw_x = (target_x - frame_center_x) / frame_center_x
                    raw_y = (frame_center_y - target_y) / frame_center_y

                    if filtered_x is None:
                        filtered_x, filtered_y = raw_x, raw_y
                    else:
                        filtered_x += self.smoothing * (raw_x - filtered_x)
                        filtered_y += self.smoothing * (raw_y - filtered_y)

                    observation = FaceObservation(
                        max(-1.0, min(1.0, filtered_x)),
                        max(-1.0, min(1.0, filtered_y)),
                        time.monotonic(),
                    )
                    with self._lock:
                        self._observation = observation

                if self.preview or self.stream:
                    cv2.line(
                        frame,
                        (int(frame_center_x) - 18, int(frame_center_y)),
                        (int(frame_center_x) + 18, int(frame_center_y)),
                        (0, 255, 255),
                        1,
                    )
                    cv2.line(
                        frame,
                        (int(frame_center_x), int(frame_center_y) - 18),
                        (int(frame_center_x), int(frame_center_y) + 18),
                        (0, 255, 255),
                        1,
                    )
                    if selected_face is not None:
                        x, y, width, height = [int(value) for value in selected_face]
                        cv2.rectangle(
                            frame,
                            (x, y),
                            (x + width, y + height),
                            (0, 255, 0),
                            2,
                        )
                        label = (
                            f"Face {face_confidence:.2f}  "
                            f"X={filtered_x:+.2f} Y={filtered_y:+.2f}"
                        )
                        color = (0, 255, 0)
                    elif hand_points:
                        connections = (
                            mediapipe.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS
                        )
                        for connection in connections:
                            cv2.line(
                                frame,
                                hand_points[connection.start],
                                hand_points[connection.end],
                                (0, 180, 255),
                                2,
                            )
                        for point in hand_points:
                            cv2.circle(frame, point, 3, (0, 255, 0), -1)
                        label = f"Hand  X={filtered_x:+.2f} Y={filtered_y:+.2f}"
                        color = (0, 255, 0)
                    else:
                        label = (
                            "Face lost - motors stopping"
                            if mode == "face"
                            else "Hand lost - motors stopping"
                        )
                        color = (0, 0, 255)
                    cv2.putText(
                        frame,
                        label,
                        (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        color,
                        2,
                        cv2.LINE_AA,
                    )
                    now = time.monotonic()
                    if self.stream and now - last_stream_at >= 1.0 / self.stream_fps:
                        encoded, jpeg = cv2.imencode(
                            ".jpg",
                            frame,
                            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                        )
                        if encoded:
                            with self._lock:
                                self._jpeg_frame = jpeg.tobytes()
                                self._frame_at = now
                            last_stream_at = now

                    if self.preview:
                        cv2.imshow(window_name, frame)
                        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                            self._stop_event.set()

        except Exception as exc:
            self._set_error(str(exc))
        finally:
            if face_detector is not None:
                try:
                    face_detector.close()
                except Exception:
                    pass
            if hand_landmarker is not None:
                try:
                    hand_landmarker.close()
                except Exception:
                    pass
            if capture is not None:
                capture.release()
            if cv2 is not None and self.preview:
                try:
                    cv2.destroyWindow(window_name)
                    cv2.waitKey(1)
                except Exception:
                    pass
            with self._lock:
                self._running = False
