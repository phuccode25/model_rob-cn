from __future__ import annotations

"""
ROBOCON BAC NINH 2026 - CAMERA FINAL
Raspberry Pi 5 + NCNN + Arduino Mega 2560

CAMERA CO DINH:
    LINE = /dev/video2
    AI   = /dev/video0

CHI MOT CAMERA ACTIVE TAI MOT THOI DIEM.

Mega -> Pi:
    LINE_ON
    AI_ON
    AI2_ON
    CAM_OFF

Pi -> Mega:
    PI_READY

    XLINE:<error_pixel>
    LOST_LINE

    TARGET:<id>
    AI_DONE

    LEFT:<id>
    RIGHT:<id>
    AI2_DONE

Bo sung BACKWARD-COMPATIBLE cho Mega can tam/tiep can:
    AIM:<xerror>,<boxHeight>
    AIM2:<xerror>,<boxHeight>

AI MAPPING - KHONG DOI:
    1 = yt   = SAMSUNG
    2 = chip = FOXCONN
    3 = al   = AMKOR
    4 = qr   = HANA MICRON VINA

Hien camera:
    SHOW_WINDOW mac dinh = TRUE.
    Khi LINE_ON: hien CAM LINE /dev/video2.
    Khi AI_ON/AI2_ON: dong LINE, mo + hien CAM AI /dev/video0.
    Khi CAM_OFF: release camera + dong window.

Chay:
    python3 robocon_pi_k1_final.py

Tat cua so neu can:
    SHOW_WINDOW=0 python3 robocon_pi_k1_final.py
"""

from pathlib import Path
from collections import deque
import os
import time

import cv2
import ncnn
import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise RuntimeError(
        "Thieu pyserial. Cai: python3 -m pip install pyserial"
    ) from exc


# ============================================================================
# MODEL
# ============================================================================

MODEL_SIZE = 416
CONF_THRESHOLD = float(os.getenv("CONF_THRESHOLD", "0.40"))
IOU_THRESHOLD = 0.45

CLASS_NAMES = [
    "al",
    "chip",
    "qr",
    "yt",
]

CLASS_TO_ID = {
    "yt": 1,      # Samsung
    "chip": 2,    # Foxconn
    "al": 3,      # Amkor
    "qr": 4,      # Hana
}

BASE_DIR = Path(__file__).resolve().parent
PARAM_PATH = BASE_DIR / "model.ncnn.param"
BIN_PATH = BASE_DIR / "model.ncnn.bin"

NCNN_THREADS = int(os.getenv("NCNN_THREADS", "4"))


# ============================================================================
# CAMERA
# ============================================================================

LINE_CAMERA_ID = int(os.getenv("LINE_CAMERA_ID", "2"))
AI_CAMERA_ID = int(os.getenv("AI_CAMERA_ID", "0"))

LINE_WIDTH = 320
LINE_HEIGHT = 240
LINE_FPS = 30

AI_WIDTH = 640
AI_HEIGHT = 480
AI_FPS = 30

# MAC DINH HIEN CAMERA.
SHOW_WINDOW = os.getenv("SHOW_WINDOW", "1") == "1"

cv2.setNumThreads(2)


# ============================================================================
# SERIAL
# ============================================================================

BAUD = int(os.getenv("BAUD_RATE", "115200"))

PREFERRED_SERIAL_PORTS = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]


# ============================================================================
# LINE
# ============================================================================

LINE_ROI_START = float(os.getenv("LINE_ROI_START", "0.62"))
LINE_THRESHOLD = int(os.getenv("LINE_THRESHOLD", "85"))  # fallback/debug
LINE_MIN_AREA = int(os.getenv("LINE_MIN_AREA", "300"))
LINE_SEND_INTERVAL = 0.04

# Lam muot toa do line, tranh giat do contour/bong.
LINE_EMA_ALPHA = float(os.getenv("LINE_EMA_ALPHA", "0.38"))
LINE_MAX_JUMP_PX = int(os.getenv("LINE_MAX_JUMP_PX", "95"))


# ============================================================================
# AI
# ============================================================================

MIN_BOX_AREA = int(os.getenv("MIN_BOX_AREA", "900"))

AI_SEND_INTERVAL = 0.04
AI_AIM_EMA_ALPHA = float(os.getenv("AI_AIM_EMA_ALPHA", "0.35"))

# Stable ID frames de chot LEFT/RIGHT.
PAIR_STABLE_FRAMES = int(
    os.getenv("PAIR_STABLE_FRAMES", "4")
)

# Single item stable frames.
SINGLE_STABLE_FRAMES = int(
    os.getenv("SINGLE_STABLE_FRAMES", "4")
)

# Deadband can tam cho camera.
AI_CENTER_DEADBAND_PX = int(
    os.getenv("AI_CENTER_DEADBAND_PX", "22")
)


# ============================================================================
# NCNN
# ============================================================================

def letterbox(image: np.ndarray, size: int = MODEL_SIZE):
    oh, ow = image.shape[:2]

    scale = min(
        size / ow,
        size / oh,
    )

    nw = int(round(ow * scale))
    nh = int(round(oh * scale))

    resized = cv2.resize(
        image,
        (nw, nh),
        interpolation=cv2.INTER_LINEAR,
    )

    pad_w = size - nw
    pad_h = size - nh

    left = pad_w // 2
    right = pad_w - left
    top = pad_h // 2
    bottom = pad_h - top

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

    rgb = cv2.cvtColor(
        padded,
        cv2.COLOR_BGR2RGB,
    )

    tensor = rgb.transpose(2, 0, 1)

    tensor = np.ascontiguousarray(
        tensor,
        dtype=np.float32,
    )

    tensor *= 1.0 / 255.0

    return tensor, scale, pad_left, pad_top


def class_aware_nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_ids: np.ndarray,
):
    keep = []

    for class_id in np.unique(class_ids):
        idx = np.flatnonzero(
            class_ids == class_id
        )

        selected = cv2.dnn.NMSBoxes(
            boxes[idx].tolist(),
            scores[idx].tolist(),
            CONF_THRESHOLD,
            IOU_THRESHOLD,
        )

        if selected is None or len(selected) == 0:
            continue

        selected = np.asarray(
            selected
        ).reshape(-1)

        keep.extend(
            idx[selected]
            .astype(int)
            .tolist()
        )

    return keep


def postprocess(
    output,
    original_shape,
    scale,
    pad_left,
    pad_top,
):
    prediction = np.asarray(
        output,
        dtype=np.float32,
    )

    prediction = np.squeeze(
        prediction
    )

    expected_channels = (
        4 + len(CLASS_NAMES)
    )

    if prediction.ndim != 2:
        raise RuntimeError(
            f"Output NCNN sai shape: {prediction.shape}"
        )

    if (
        prediction.shape[0]
        ==
        expected_channels
    ):
        prediction = prediction.T

    elif (
        prediction.shape[1]
        !=
        expected_channels
    ):
        raise RuntimeError(
            f"Khong nhan dang duoc output NCNN: {prediction.shape}"
        )

    class_scores = prediction[
        :,
        4:4 + len(CLASS_NAMES),
    ]

    class_ids = np.argmax(
        class_scores,
        axis=1,
    ).astype(np.int32)

    rows = np.arange(
        prediction.shape[0]
    )

    scores = class_scores[
        rows,
        class_ids,
    ]

    mask = (
        scores
        >=
        CONF_THRESHOLD
    )

    if not np.any(mask):
        return []

    xywh = prediction[
        mask,
        :4,
    ]

    scores = scores[
        mask
    ].astype(np.float32)

    class_ids = class_ids[
        mask
    ]

    cx = xywh[:, 0]
    cy = xywh[:, 1]
    bw = xywh[:, 2]
    bh = xywh[:, 3]

    x1 = (
        cx - bw * 0.5 - pad_left
    ) / scale

    y1 = (
        cy - bh * 0.5 - pad_top
    ) / scale

    x2 = (
        cx + bw * 0.5 - pad_left
    ) / scale

    y2 = (
        cy + bh * 0.5 - pad_top
    ) / scale

    oh, ow = original_shape[:2]

    x1 = np.clip(x1, 0, ow - 1)
    y1 = np.clip(y1, 0, oh - 1)
    x2 = np.clip(x2, 0, ow - 1)
    y2 = np.clip(y2, 0, oh - 1)

    widths = x2 - x1
    heights = y2 - y1

    valid = (
        (widths > 1)
        &
        (heights > 1)
    )

    if not np.any(valid):
        return []

    boxes = np.stack(
        (
            x1,
            y1,
            widths,
            heights,
        ),
        axis=1,
    )

    boxes = boxes[
        valid
    ].astype(np.int32)

    scores = scores[
        valid
    ]

    class_ids = class_ids[
        valid
    ]

    selected = class_aware_nms(
        boxes,
        scores,
        class_ids,
    )

    detections = []

    for i in selected:
        x, y, w, h = boxes[i].tolist()

        if w * h < MIN_BOX_AREA:
            continue

        detections.append({
            "box": [x, y, w, h],
            "score": float(scores[i]),
            "class_id": int(class_ids[i]),
        })

    return detections


def load_ncnn():
    if not PARAM_PATH.exists():
        raise FileNotFoundError(
            f"Khong thay {PARAM_PATH}"
        )

    if not BIN_PATH.exists():
        raise FileNotFoundError(
            f"Khong thay {BIN_PATH}"
        )

    net = ncnn.Net()

    net.opt.use_vulkan_compute = False
    net.opt.num_threads = NCNN_THREADS

    r1 = net.load_param(
        str(PARAM_PATH)
    )

    r2 = net.load_model(
        str(BIN_PATH)
    )

    if r1 != 0 or r2 != 0:
        raise RuntimeError(
            f"Load NCNN fail param={r1} model={r2}"
        )

    input_names = net.input_names()
    output_names = net.output_names()

    if not input_names or not output_names:
        raise RuntimeError(
            "Model khong co input/output."
        )

    input_name = input_names[0]
    output_name = sorted(
        output_names
    )[0]

    print(
        "NCNN OK:",
        f"input={input_name}",
        f"output={output_name}",
        f"classes={CLASS_NAMES}",
        flush=True,
    )

    return net, input_name, output_name


def run_yolo(
    net,
    input_name,
    output_name,
    frame,
):
    tensor, scale, pad_left, pad_top = preprocess(
        frame
    )

    input_mat = ncnn.Mat(
        tensor
    ).clone()

    with net.create_extractor() as ex:
        ex.input(
            input_name,
            input_mat,
        )

        rc, output = ex.extract(
            output_name
        )

    if rc != 0:
        raise RuntimeError(
            f"NCNN inference error={rc}"
        )

    return postprocess(
        output,
        frame.shape,
        scale,
        pad_left,
        pad_top,
    )


# ============================================================================
# SERIAL
# ============================================================================

def find_serial_port():
    requested = os.getenv(
        "SERIAL_PORT",
        "",
    ).strip()

    ports = [
        p.device
        for p in list_ports.comports()
    ]

    print(
        "Serial hien co:",
        ports,
        flush=True,
    )

    if requested:
        if requested in ports:
            return requested

        print(
            f"Khong co {requested}",
            flush=True,
        )

    for p in PREFERRED_SERIAL_PORTS:
        if p in ports:
            return p

    return None


def open_serial():
    port = find_serial_port()

    if port is None:
        raise RuntimeError(
            "Khong tim thay Mega."
        )

    conn = serial.Serial(
        port,
        BAUD,
        timeout=0.01,
        write_timeout=0.10,
    )

    # Mega reset khi mo USB.
    time.sleep(2.0)

    conn.reset_input_buffer()
    conn.reset_output_buffer()

    print(
        f"MEGA OK: {port} @ {BAUD}",
        flush=True,
    )

    return conn


def send_line(conn, text: str):
    conn.write(
        (text.strip() + "\n")
        .encode("utf-8")
    )


# ============================================================================
# CAMERA MANAGER
# ============================================================================

class CameraManager:
    def __init__(self):
        self.cap = None
        self.mode = "OFF"
        self.window_name = ""

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        if SHOW_WINDOW:
            cv2.destroyAllWindows()
            cv2.waitKey(1)

        self.mode = "OFF"
        self.window_name = ""

    def _open(
        self,
        camera_id: int,
        width: int,
        height: int,
        fps: int,
        name: str,
    ):
        self.close()

        for fmt in ("YUYV", "MJPG"):
            print(
                f"Thu {name} /dev/video{camera_id} {fmt}",
                flush=True,
            )

            cap = cv2.VideoCapture(
                camera_id,
                cv2.CAP_V4L2,
            )

            if not cap.isOpened():
                cap.release()
                continue

            cap.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(
                    *fmt
                ),
            )

            cap.set(
                cv2.CAP_PROP_FRAME_WIDTH,
                width,
            )

            cap.set(
                cv2.CAP_PROP_FRAME_HEIGHT,
                height,
            )

            cap.set(
                cv2.CAP_PROP_FPS,
                fps,
            )

            cap.set(
                cv2.CAP_PROP_BUFFERSIZE,
                1,
            )

            ok = False
            frame = None

            for _ in range(20):
                ok, frame = cap.read()

                if ok and frame is not None:
                    break

                time.sleep(0.03)

            if ok and frame is not None:
                self.cap = cap
                self.window_name = name

                print(
                    f"{name} OK /dev/video{camera_id} "
                    f"{frame.shape[1]}x{frame.shape[0]}",
                    flush=True,
                )

                return True

            cap.release()

        return False

    def open_line(self):
        ok = self._open(
            LINE_CAMERA_ID,
            LINE_WIDTH,
            LINE_HEIGHT,
            LINE_FPS,
            "CAM LINE /dev/video2",
        )

        if ok:
            self.mode = "LINE"

        return ok

    def open_ai(self, mode: str):
        ok = self._open(
            AI_CAMERA_ID,
            AI_WIDTH,
            AI_HEIGHT,
            AI_FPS,
            "CAM AI /dev/video0",
        )

        if ok:
            self.mode = mode

        return ok

    def read(self):
        if self.cap is None:
            return False, None

        return self.cap.read()

    def show(self, frame):
        if (
            SHOW_WINDOW
            and
            frame is not None
        ):
            cv2.imshow(
                self.window_name,
                frame,
            )

            key = (
                cv2.waitKey(1)
                &
                0xFF
            )

            return (
                key == ord("q")
                or
                key == 27
            )

        return False


# ============================================================================
# LINE PROCESS
# ============================================================================

def process_black_line(frame):
    h, w = frame.shape[:2]

    roi_y = int(
        h * LINE_ROI_START
    )

    # Bo mot it mep anh de giam bat nham vien san/bong banh.
    margin_x = int(w * 0.06)

    roi = frame[
        roi_y:h,
        margin_x:w - margin_x,
    ]

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY,
    )

    gray = cv2.GaussianBlur(
        gray,
        (5, 5),
        0,
    )

    # OTSU tu can nguong theo anh sang, on hon threshold co dinh 85.
    _, mask = cv2.threshold(
        gray,
        0,
        255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
    )

    # OPEN nho de khong xoa mat line mong; CLOSE de noi vet line bi dut.
    kernel_open = np.ones(
        (3, 3),
        np.uint8,
    )
    kernel_close = np.ones(
        (5, 5),
        np.uint8,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel_open,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel_close,
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    candidates = []

    prev_x = getattr(
        process_black_line,
        "_last_x",
        w * 0.5,
    )

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < LINE_MIN_AREA:
            continue

        m = cv2.moments(contour)

        if m["m00"] == 0:
            continue

        x_roi = (
            m["m10"]
            /
            m["m00"]
        )

        x = x_roi + margin_x

        y_roi = (
            m["m01"]
            /
            m["m00"]
        )

        # Uu tien contour gan vi tri line frame truoc.
        # Dien tich lon van duoc uu tien nhe, nhung khong con "lon nhat la thang".
        score = (
            abs(x - prev_x)
            -
            0.0025 * min(area, 5000.0)
        )

        candidates.append(
            (
                score,
                x,
                y_roi,
                area,
            )
        )

    if not candidates:
        return None, mask, roi_y, None

    candidates.sort(
        key=lambda item: item[0]
    )

    _, raw_x, y_roi, area = candidates[0]

    # Chan nhay dot ngot sang bong/vet den khac.
    if (
        hasattr(process_black_line, "_last_x")
        and
        abs(raw_x - prev_x) > LINE_MAX_JUMP_PX
    ):
        raw_x = prev_x

    filtered_x = (
        LINE_EMA_ALPHA * raw_x
        +
        (1.0 - LINE_EMA_ALPHA) * prev_x
    )

    process_black_line._last_x = filtered_x

    x = int(round(filtered_x))
    y = int(round(y_roi + roi_y))
    error = x - w // 2

    return error, mask, roi_y, {
        "x": x,
        "y": y,
        "area": area,
    }


# ============================================================================
# AI HELPERS
# ============================================================================

def center_of(det):
    x, y, w, h = det["box"]

    return (
        x + w * 0.5,
        y + h * 0.5,
    )


def class_name(det):
    return CLASS_NAMES[
        det["class_id"]
    ]


def cargo_id(det):
    return CLASS_TO_ID[
        class_name(det)
    ]


def valid_ai_detections(
    detections,
    frame_shape,
):
    # Khong loai box chi vi cham mep anh.
    # Khi robot den gan, box rat de cham/cat mep; loai no se lam mat AIM2
    # dung luc can dung robot nhat.
    result = []

    for det in detections:
        x, y, bw, bh = det["box"]

        if bw < 8 or bh < 8:
            continue

        result.append(det)

    return result


def select_single(
    detections,
    frame_shape,
):
    valid = valid_ai_detections(
        detections,
        frame_shape,
    )

    if not valid:
        return None

    frame_cx = (
        frame_shape[1]
        *
        0.5
    )

    valid.sort(
        key=lambda d:
        (
            abs(
                center_of(d)[0]
                -
                frame_cx
            )
            -
            0.00015
            *
            (
                d["box"][2]
                *
                d["box"][3]
            )
        )
    )

    return valid[0]


def select_pair(
    detections,
    frame_shape,
):
    """
    Chon 2 kien hop ly nhat cho cap dang gap.

    Khong thay doi mapping.
    Neu camera thay >2 box, uu tien 2 box gan tam camera nhat,
    sau do sap xep LEFT / RIGHT theo toa do x.

    Day la quy tac chon cap; neu co khi cua ban can ROI/tang cu the,
    chi sua ham nay.
    """
    valid = valid_ai_detections(
        detections,
        frame_shape,
    )

    if len(valid) < 2:
        return None

    frame_cx = (
        frame_shape[1]
        *
        0.5
    )

    valid.sort(
        key=lambda d:
        abs(
            center_of(d)[0]
            -
            frame_cx
        )
    )

    pair = valid[:2]

    pair.sort(
        key=lambda d:
        center_of(d)[0]
    )

    return pair[0], pair[1]


def pair_aim(
    left,
    right,
    frame_shape,
):
    lx, _ = center_of(left)
    rx, _ = center_of(right)

    pair_center_x = (
        lx + rx
    ) * 0.5

    frame_center_x = (
        frame_shape[1]
        *
        0.5
    )

    xerr = int(
        round(
            pair_center_x
            -
            frame_center_x
        )
    )

    # Box height trung binh lam proxy khoang cach.
    h1 = left["box"][3]
    h2 = right["box"][3]

    avg_h = int(
        round(
            (h1 + h2)
            *
            0.5
        )
    )

    return xerr, avg_h


# ============================================================================
# DEBUG DRAW
# ============================================================================

def draw_line_debug(
    frame,
    error,
    roi_y,
    debug,
):
    h, w = frame.shape[:2]

    cv2.line(
        frame,
        (w // 2, roi_y),
        (w // 2, h),
        (255, 255, 0),
        2,
    )

    cv2.line(
        frame,
        (0, roi_y),
        (w, roi_y),
        (255, 0, 0),
        2,
    )

    if debug is not None:
        cv2.circle(
            frame,
            (
                debug["x"],
                debug["y"],
            ),
            8,
            (0, 0, 255),
            -1,
        )

    cv2.putText(
        frame,
        f"LINE error={error}",
        (15, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )


def draw_ai_detections(
    frame,
    detections,
    selected=None,
    aim=None,
):
    h, w = frame.shape[:2]

    cv2.line(
        frame,
        (w // 2, 0),
        (w // 2, h),
        (255, 255, 0),
        2,
    )

    selected_ids = set()

    if selected is not None:
        if isinstance(selected, tuple):
            selected_ids = {
                id(selected[0]),
                id(selected[1]),
            }
        else:
            selected_ids = {
                id(selected)
            }

    for det in detections:
        x, y, bw, bh = det["box"]

        name = class_name(det)
        cid = CLASS_TO_ID[name]

        color = (
            (0, 0, 255)
            if id(det) in selected_ids
            else (0, 255, 0)
        )

        cv2.rectangle(
            frame,
            (x, y),
            (x + bw, y + bh),
            color,
            2,
        )

        cv2.putText(
            frame,
            f"{name}/ID{cid} {det['score']:.2f}",
            (
                x,
                max(18, y - 5),
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            2,
        )

    if aim is not None:
        xerr, box_h = aim

        cv2.putText(
            frame,
            f"AIM x={xerr} boxH={box_h}",
            (15, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )


# ============================================================================
# COMMAND HANDLER
# ============================================================================

def reset_ai_state(state):
    state["single_history"].clear()
    state["pair_history"].clear()
    state["single_done"] = False
    state["pair_done"] = False
    state["aim_x_ema"] = None
    state["aim_h_ema"] = None


def handle_command(
    command,
    state,
    camera: CameraManager,
):
    command = command.strip()

    if not command:
        return

    print(
        "MEGA -> PI:",
        command,
        flush=True,
    )

    head = command.upper()

    if head == "LINE_ON":
        reset_ai_state(state)

        if not camera.open_line():
            raise RuntimeError(
                "Khong mo duoc camera LINE /dev/video2"
            )

        print(
            "MODE -> LINE",
            flush=True,
        )
        return

    if head == "AI_ON":
        reset_ai_state(state)

        if not camera.open_ai("AI"):
            raise RuntimeError(
                "Khong mo duoc camera AI /dev/video0"
            )

        print(
            "MODE -> AI",
            flush=True,
        )
        return

    if head == "AI2_ON":
        reset_ai_state(state)

        if not camera.open_ai("AI2"):
            raise RuntimeError(
                "Khong mo duoc camera AI /dev/video0"
            )

        print(
            "MODE -> AI2",
            flush=True,
        )
        return

    if head == "CAM_OFF":
        camera.close()

        print(
            "MODE -> OFF",
            flush=True,
        )
        return

    # DBG/noi dung khac:
    # chi print, khong tac dong camera.


# ============================================================================
# MAIN
# ============================================================================

def main():
    print("=" * 68)
    print("ROBOCON O2 - PI CAMERA FINAL")
    print("=" * 68)

    net, input_name, output_name = load_ncnn()
    conn = open_serial()

    camera = CameraManager()

    state = {
        "single_history": deque(
            maxlen=SINGLE_STABLE_FRAMES
        ),
        "pair_history": deque(
            maxlen=PAIR_STABLE_FRAMES
        ),
        "single_done": False,
        "pair_done": False,
        "aim_x_ema": None,
        "aim_h_ema": None,
    }

    serial_buffer = ""

    last_line_send = 0.0
    last_ai_send = 0.0
    last_log = 0.0

    send_line(
        conn,
        "PI_READY",
    )

    print(
        "PI_READY da gui.",
        flush=True,
    )

    print(
        f"LINE=/dev/video{LINE_CAMERA_ID}",
        flush=True,
    )

    print(
        f"AI=/dev/video{AI_CAMERA_ID}",
        flush=True,
    )

    print(
        f"SHOW_WINDOW={SHOW_WINDOW}",
        flush=True,
    )

    try:
        while True:
            # ---------------------------------------------------------------
            # SERIAL
            # ---------------------------------------------------------------
            while conn.in_waiting > 0:
                chunk = conn.read(
                    conn.in_waiting
                ).decode(
                    "utf-8",
                    errors="ignore",
                )

                serial_buffer += chunk

                while "\n" in serial_buffer:
                    line, serial_buffer = serial_buffer.split(
                        "\n",
                        1,
                    )

                    handle_command(
                        line,
                        state,
                        camera,
                    )

            if camera.mode == "OFF":
                time.sleep(0.005)
                continue

            ok, frame = camera.read()

            if not ok or frame is None:
                if camera.mode == "LINE":
                    send_line(
                        conn,
                        "LOST_LINE",
                    )

                time.sleep(0.01)
                continue

            now = time.monotonic()

            # ---------------------------------------------------------------
            # LINE
            # ---------------------------------------------------------------
            if camera.mode == "LINE":
                (
                    error,
                    mask,
                    roi_y,
                    debug,
                ) = process_black_line(
                    frame
                )

                if (
                    now - last_line_send
                    >=
                    LINE_SEND_INTERVAL
                ):
                    if error is None:
                        send_line(
                            conn,
                            "LOST_LINE",
                        )
                    else:
                        send_line(
                            conn,
                            f"XLINE:{error}",
                        )

                    last_line_send = now

                if SHOW_WINDOW:
                    draw_line_debug(
                        frame,
                        (
                            error
                            if error is not None
                            else 0
                        ),
                        roi_y,
                        debug,
                    )

            # ---------------------------------------------------------------
            # AI / AI2
            # ---------------------------------------------------------------
            else:
                detections = run_yolo(
                    net,
                    input_name,
                    output_name,
                    frame,
                )

                if camera.mode == "AI":
                    selected = select_single(
                        detections,
                        frame.shape,
                    )

                    aim = None

                    if selected is not None:
                        cx, _ = center_of(
                            selected
                        )

                        xerr = int(
                            round(
                                cx
                                -
                                frame.shape[1] * 0.5
                            )
                        )

                        box_h = int(
                            selected["box"][3]
                        )

                        aim = (
                            xerr,
                            box_h,
                        )

                        if (
                            now - last_ai_send
                            >=
                            AI_SEND_INTERVAL
                        ):
                            send_line(
                                conn,
                                f"AIM:{xerr},{box_h}",
                            )

                            last_ai_send = now

                        state[
                            "single_history"
                        ].append(
                            cargo_id(selected)
                        )

                        hist = list(
                            state["single_history"]
                        )

                        if (
                            len(hist)
                            ==
                            SINGLE_STABLE_FRAMES
                            and
                            len(set(hist))
                            ==
                            1
                            and
                            not state["single_done"]
                        ):
                            cid = hist[0]

                            send_line(
                                conn,
                                f"TARGET:{cid}",
                            )

                            send_line(
                                conn,
                                "AI_DONE",
                            )

                            state[
                                "single_done"
                            ] = True

                    else:
                        state[
                            "single_history"
                        ].clear()

                    if SHOW_WINDOW:
                        draw_ai_detections(
                            frame,
                            detections,
                            selected=selected,
                            aim=aim,
                        )

                elif camera.mode == "AI2":
                    pair = select_pair(
                        detections,
                        frame.shape,
                    )

                    aim = None

                    if pair is not None:
                        left, right = pair

                        raw_xerr, raw_box_h = pair_aim(
                            left,
                            right,
                            frame.shape,
                        )

                        if state["aim_x_ema"] is None:
                            state["aim_x_ema"] = float(raw_xerr)
                            state["aim_h_ema"] = float(raw_box_h)
                        else:
                            a = AI_AIM_EMA_ALPHA
                            state["aim_x_ema"] = (
                                a * raw_xerr
                                +
                                (1.0 - a) * state["aim_x_ema"]
                            )
                            state["aim_h_ema"] = (
                                a * raw_box_h
                                +
                                (1.0 - a) * state["aim_h_ema"]
                            )

                        xerr = int(round(state["aim_x_ema"]))
                        box_h = int(round(state["aim_h_ema"]))
                        aim = (xerr, box_h)

                        if (
                            now - last_ai_send
                            >=
                            AI_SEND_INTERVAL
                        ):
                            send_line(
                                conn,
                                f"AIM2:{xerr},{box_h}",
                            )

                            last_ai_send = now

                        pair_ids = (
                            cargo_id(left),
                            cargo_id(right),
                        )

                        state[
                            "pair_history"
                        ].append(
                            pair_ids
                        )

                        hist = list(
                            state["pair_history"]
                        )

                        if (
                            len(hist)
                            ==
                            PAIR_STABLE_FRAMES
                            and
                            len(set(hist))
                            ==
                            1
                            and
                            not state["pair_done"]
                        ):
                            left_id, right_id = hist[0]

                            send_line(
                                conn,
                                f"LEFT:{left_id}",
                            )

                            send_line(
                                conn,
                                f"RIGHT:{right_id}",
                            )

                            send_line(
                                conn,
                                "AI2_DONE",
                            )

                            print(
                                "AI2 CHOT:",
                                f"LEFT={left_id}",
                                f"RIGHT={right_id}",
                                flush=True,
                            )

                            state[
                                "pair_done"
                            ] = True

                    else:
                        state[
                            "pair_history"
                        ].clear()

                    if SHOW_WINDOW:
                        draw_ai_detections(
                            frame,
                            detections,
                            selected=pair,
                            aim=aim,
                        )

            # ---------------------------------------------------------------
            # WINDOW
            # ---------------------------------------------------------------
            if camera.show(frame):
                break

            if now - last_log >= 0.8:
                print(
                    "MODE=",
                    camera.mode,
                    "detections=",
                    (
                        len(detections)
                        if camera.mode in ("AI", "AI2")
                        else "-"
                    ),
                    flush=True,
                )

                last_log = now

    except KeyboardInterrupt:
        print(
            "\nDung Ctrl+C.",
            flush=True,
        )

    finally:
        try:
            send_line(
                conn,
                "PI_STOP",
            )
        except Exception:
            pass

        camera.close()

        conn.close()

        net.clear()


if __name__ == "__main__":
    main()
