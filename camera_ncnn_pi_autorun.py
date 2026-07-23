from pathlib import Path
import os
import time

import cv2
import ncnn
import numpy as np


MODEL_SIZE = 416
CONF_THRESHOLD = 0.40
IOU_THRESHOLD = 0.45

# Mặc định chạy không màn hình để dùng với systemd khi thi.
# Chạy thủ công có cửa sổ bằng:
# SHOW_WINDOW=1 python camera_ncnn_pi_autorun.py
SHOW_WINDOW = os.getenv("SHOW_WINDOW", "0") == "1"

# Raspberry Pi thường nhận webcam USB ở /dev/video0.
CAMERA_IDS = (0, 1, 2, 3)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

CLASS_NAMES = [
    "al",
    "chip",
    "qr",
    "yt",
]

BASE_DIR = Path(__file__).resolve().parent
PARAM_PATH = BASE_DIR / "model.ncnn.param"
BIN_PATH = BASE_DIR / "model.ncnn.bin"


def letterbox(image: np.ndarray, size: int = 416):
    """
    Resize ảnh nhưng giữ nguyên tỷ lệ.
    Phần còn thiếu được đệm bằng màu xám 114.
    """
    original_height, original_width = image.shape[:2]

    scale = min(
        size / original_width,
        size / original_height,
    )

    new_width = int(round(original_width * scale))
    new_height = int(round(original_height * scale))

    resized = cv2.resize(
        image,
        (new_width, new_height),
        interpolation=cv2.INTER_LINEAR,
    )

    pad_width = size - new_width
    pad_height = size - new_height

    left = pad_width // 2
    right = pad_width - left
    top = pad_height // 2
    bottom = pad_height - top

    padded = cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )

    return padded, scale, left, top


def preprocess(frame: np.ndarray):
    padded, scale, pad_left, pad_top = letterbox(
        frame,
        MODEL_SIZE,
    )

    # OpenCV dùng BGR, model dùng RGB
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)

    # HWC -> CHW, uint8 -> float32, chuẩn hóa về [0, 1]
    tensor = rgb.transpose(2, 0, 1)
    tensor = np.ascontiguousarray(tensor, dtype=np.float32)
    tensor /= 255.0

    return tensor, scale, pad_left, pad_top


def class_aware_nms(boxes, scores, class_ids):
    """
    Chạy NMS riêng cho từng lớp để các lớp khác nhau
    không loại nhầm bounding box của nhau.
    """
    keep = []

    for class_id in set(class_ids):
        class_indices = [
            index
            for index, value in enumerate(class_ids)
            if value == class_id
        ]

        class_boxes = [boxes[index] for index in class_indices]
        class_scores = [scores[index] for index in class_indices]

        selected = cv2.dnn.NMSBoxes(
            class_boxes,
            class_scores,
            CONF_THRESHOLD,
            IOU_THRESHOLD,
        )

        if len(selected) == 0:
            continue

        selected = np.asarray(selected).reshape(-1)

        for local_index in selected:
            keep.append(class_indices[int(local_index)])

    return keep


def postprocess(
    output,
    original_shape,
    scale,
    pad_left,
    pad_top,
):
    prediction = np.asarray(output, dtype=np.float32)
    prediction = np.squeeze(prediction)

    expected_channels = 4 + len(CLASS_NAMES)  # x, y, w, h + 4 lớp

    if prediction.ndim != 2:
        raise RuntimeError(
            f"Đầu ra model không đúng 2 chiều: {prediction.shape}"
        )

    # NCNN thường trả về dạng (8, 3549).
    # Chuyển thành (3549, 8).
    if prediction.shape[0] == expected_channels:
        prediction = prediction.T
    elif prediction.shape[1] != expected_channels:
        raise RuntimeError(
            "Không nhận dạng được cấu trúc output. "
            f"Shape hiện tại: {prediction.shape}, "
            f"mong đợi một chiều bằng {expected_channels}."
        )

    original_height, original_width = original_shape[:2]

    boxes = []
    scores = []
    class_ids = []

    for row in prediction:
        class_scores = row[4 : 4 + len(CLASS_NAMES)]

        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])

        if confidence < CONF_THRESHOLD:
            continue

        center_x, center_y, box_width, box_height = row[:4]

        # Tọa độ trong ảnh 416 × 416
        x1 = center_x - box_width / 2
        y1 = center_y - box_height / 2
        x2 = center_x + box_width / 2
        y2 = center_y + box_height / 2

        # Bỏ phần padding và đưa về kích thước camera gốc
        x1 = (x1 - pad_left) / scale
        y1 = (y1 - pad_top) / scale
        x2 = (x2 - pad_left) / scale
        y2 = (y2 - pad_top) / scale

        x1 = max(0, min(original_width - 1, x1))
        y1 = max(0, min(original_height - 1, y1))
        x2 = max(0, min(original_width - 1, x2))
        y2 = max(0, min(original_height - 1, y2))

        width = x2 - x1
        height = y2 - y1

        if width <= 1 or height <= 1:
            continue

        boxes.append([
            int(x1),
            int(y1),
            int(width),
            int(height),
        ])

        scores.append(confidence)
        class_ids.append(class_id)

    if not boxes:
        return []

    selected_indices = class_aware_nms(
        boxes,
        scores,
        class_ids,
    )

    detections = []

    for index in selected_indices:
        detections.append({
            "box": boxes[index],
            "score": scores[index],
            "class_id": class_ids[index],
        })

    return detections


def draw_detections(frame, detections, fps):
    for detection in detections:
        x, y, width, height = detection["box"]
        score = detection["score"]
        class_id = detection["class_id"]

        x2 = x + width
        y2 = y + height

        label = f"{CLASS_NAMES[class_id]} {score:.2f}"

        cv2.rectangle(
            frame,
            (x, y),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        label_width, label_height = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            2,
        )[0]

        text_y = max(y, label_height + 8)

        cv2.rectangle(
            frame,
            (x, text_y - label_height - 8),
            (x + label_width + 8, text_y),
            (0, 255, 0),
            -1,
        )

        cv2.putText(
            frame,
            label,
            (x + 4, text_y - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def open_camera():
    """Mở webcam USB trên Raspberry Pi bằng backend V4L2."""
    for camera_id in CAMERA_IDS:
        print(f"Đang thử camera ID {camera_id}...", flush=True)

        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)

        if not camera.isOpened():
            camera.release()
            continue

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Camera có thể cần vài frame đầu để ổn định.
        success = False
        frame = None

        for _ in range(10):
            success, frame = camera.read()
            if success and frame is not None:
                break
            time.sleep(0.1)

        if success and frame is not None:
            print(
                f"Đã mở camera ID {camera_id}: "
                f"{frame.shape[1]}x{frame.shape[0]}",
                flush=True,
            )
            return camera

        print(
            f"Camera ID {camera_id} mở được nhưng không đọc được ảnh.",
            flush=True,
        )
        camera.release()

    return None


def main():
    if not PARAM_PATH.exists():
        raise FileNotFoundError(
            f"Không tìm thấy: {PARAM_PATH}"
        )

    if not BIN_PATH.exists():
        raise FileNotFoundError(
            f"Không tìm thấy: {BIN_PATH}"
        )

    print("Đang tải model NCNN...")
    print("Param:", PARAM_PATH)
    print("Bin:", BIN_PATH)

    net = ncnn.Net()
    net.opt.use_vulkan_compute = False
    net.opt.num_threads = 4

    param_result = net.load_param(str(PARAM_PATH))
    model_result = net.load_model(str(BIN_PATH))

    if param_result != 0:
        raise RuntimeError(
            f"Không load được file param, mã lỗi: {param_result}"
        )

    if model_result != 0:
        raise RuntimeError(
            f"Không load được file bin, mã lỗi: {model_result}"
        )

    input_name = net.input_names()[0]
    output_name = sorted(net.output_names())[0]

    print("Input model:", input_name)
    print("Output model:", output_name)
    print("Model đã load thành công.")

    camera = open_camera()

    if camera is None:
        raise RuntimeError(
            "Không mở được webcam USB hoặc camera laptop. "
            "Hãy đóng Camera, Zoom, Google Meet rồi chạy lại."
        )

    if SHOW_WINDOW:
        print("Chế độ xem hình: bật. Nhấn Q hoặc ESC để thoát.")
    else:
        print("Chế độ thi tự động: không mở cửa sổ camera.")

    first_frame = True
    last_detection_text = ""
    last_log_time = 0.0

    try:
        while True:
            success, frame = camera.read()

            if not success:
                print("Không đọc được hình ảnh từ camera.")
                break

            start_time = time.perf_counter()

            tensor, scale, pad_left, pad_top = preprocess(frame)

            input_mat = ncnn.Mat(tensor).clone()

            with net.create_extractor() as extractor:
                extractor.input(input_name, input_mat)

                result_code, output = extractor.extract(output_name)

            if result_code != 0:
                raise RuntimeError(
                    f"Lỗi chạy inference NCNN: {result_code}"
                )

            if first_frame:
                print(
                    "Output shape:",
                    np.asarray(output).shape,
                )
                first_frame = False

            detections = postprocess(
                output,
                frame.shape,
                scale,
                pad_left,
                pad_top,
            )

            elapsed = time.perf_counter() - start_time
            fps = 1.0 / elapsed if elapsed > 0 else 0.0

            # Đây là vị trí nối AI với điều khiển robot.
            # Hiện tại chương trình in lớp nhận diện tốt nhất.
            # Sau này có thể gửi tên lớp qua UART/USB Serial cho STM32/Arduino.
            if detections:
                best = max(detections, key=lambda item: item["score"])
                class_name = CLASS_NAMES[best["class_id"]]
                detection_text = f"{class_name}:{best['score']:.2f}"
            else:
                detection_text = "none"

            now = time.monotonic()
            if (
                detection_text != last_detection_text
                or now - last_log_time >= 2.0
            ):
                print(
                    f"DETECTION={detection_text} FPS={fps:.1f}",
                    flush=True,
                )
                last_detection_text = detection_text
                last_log_time = now

            if SHOW_WINDOW:
                draw_detections(frame, detections, fps)

                cv2.imshow(
                    "ROBOCON YOLO26 NCNN - Q to quit",
                    frame,
                )

                key = cv2.waitKey(1) & 0xFF

                if key == ord("q") or key == 27:
                    break

    finally:
        camera.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        net.clear()


if __name__ == "__main__":
    main()