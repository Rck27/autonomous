"""

Usage:
    python live_inference.py                        # screen (default)
    python live_inference.py --source 0             # webcam index 0
    python live_inference.py --source video.mp4     # video file
    python live_inference.py --source rtsp://...    # RTSP stream

Controls:
    Q or ESC  → quit
    P         → pause / resume
    S         → save screenshot
"""

import argparse
import time
import cv2
import numpy as np
from pathlib import Path
from collections import deque
from ultralytics import YOLO

import zmq, msgpack_numpy as m

import memory_config as cfg



from mss import mss

MODEL_PATH   = "rdk/model/best.pt"      
CONF_THRESH  = 0.40            # minimum confidence to show a box
IOU_THRESH   = 0.45            # NMS IoU threshold
IMG_SIZE     = 640             # inference resolution
DEVICE       = ""              # "" = auto (GPU if available, else CPU)
FPS_WINDOW   = 30              # rolling average over N frames


def zmq_create():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(cfg.IPC_PATH)
    # socket.setsockopt_string(zmq.SNDHWM, "1")

    return socket

def zmq_close(socket):
    socket.unbind(cfg.IPC_PATH)
    socket.close()

def zmq_send(socket, data):
    packed = m.packb(data)
    socket.send(packed)


def draw_box(frame, x1, y1, x2, y2, label: str, conf: float, class_id: int):
    color = (0, 0, 0)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    text  = f"{label}  {conf:.0%}"
    font  = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thick = 1
    (tw, th), _ = cv2.getTextSize(text, font, scale, thick)

    # header background
    by1 = max(0, y1 - th - 8)
    by2 = y1
    cv2.rectangle(frame, (x1, by1), (x1 + tw + 8, by2), color, -1)
    cv2.putText(frame, text, (x1 + 4, by2 - 4),
                font, scale, (0, 0, 0), thick, cv2.LINE_AA)



def draw_hud(frame, fps: float, det_count: int, paused: bool,
             model_name: str, width: int, height: int):
    """Draw semi-transparent HUD bar at the top of the frame."""
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 38), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    font  = cv2.FONT_HERSHEY_SIMPLEX
    left  = f"Model: {model_name}   {width}x{height}"
    right = f"{'⏸ PAUSED   ' if paused else ''}FPS: {fps:5.1f}   Detections: {det_count}"

    cv2.putText(frame, left,  (10, 26), font, 0.58, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(frame, right, (w - 10 - cv2.getTextSize(right, font, 0.58, 1)[0][0], 26),
                font, 0.58, (0, 220, 255), 1, cv2.LINE_AA)


def draw_class_summary(frame, class_counts: dict, names: dict):
    """Draw a small legend showing detected classes and counts."""
    if not class_counts:
        return
    h, w = frame.shape[:2]
    x, y = 10, 50
    for cls_id, cnt in sorted(class_counts.items()):
        label = names.get(cls_id, str(cls_id))
        text  = f"{label}: {cnt}"
        color = (0,0,0)
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 2, cv2.LINE_AA)
        y += 22

from matplotlib import pyplot as plt
def color_det(x1, y1, x2, y2, image, class_id):
    img = image[y1:y2, x1:x2]
    img = cv2.resize(img, (64, 64), interpolation=cv2.INTER_AREA)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    pixels = hsv.reshape(-1, 3)
    h_bins = (pixels[:, 0] // 10).astype(np.uint8)  # quantize hue
    dominant_bin = np.bincount(h_bins).argmax()
    
    mask = h_bins == dominant_bin
    dominant_hsv = pixels[mask].mean(axis=0)
    
    h, s, v = dominant_hsv
    if s < 40:
        if v < 60:   return cfg.COLOR_DICT["black"]
        if v > 200:  return cfg.COLOR_DICT["white"]
        return cfg.COLOR_DICT["gray"]
    
    if h < 10 or h >= 160:   return cfg.COLOR_DICT["red"]
    if 10 <= h < 25:         return cfg.COLOR_DICT["orange"]
    if 25 <= h < 35:         return cfg.COLOR_DICT["yellow"]
    if 35 <= h < 85:         return cfg.COLOR_DICT["green"]
    if 85 <= h < 130:        return cfg.COLOR_DICT["blue"]
    if 130 <= h < 160:       return cfg.COLOR_DICT["purple"]
    
    return cfg.COLOR_DICT["unknown"]



def run(source, model_path, conf, iou, imgsz, device, gui):

    socket = zmq_create()
    data = np.zeros(1, dtype=cfg.meta_dtype)

    model  = YOLO(model_path)
    names  = model.names
    mname  = Path(model_path).stem

    src = int(source) if str(source).isdigit() else source

    is_screen = (source == "scr")
    sct = None
    monitor = None
    cap = None

    if is_screen:
        print("Capturing screen...")
        sct     = mss()
        monitor = sct.monitors[1]  # primary monitor
        frame_w = monitor['width']
        frame_h = monitor['height']
        print(f"Screen size: {frame_w}x{frame_h}")
        # exit(0)
    else:
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open source: {source}")
        frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Screen size: {frame_w}x{frame_h}")
        # exit(0)

    fps_times: deque = deque(maxlen=FPS_WINDOW)
    paused       = False
    last_results = None
    screenshot_idx = 0

    print(f"\n  Model   : {model_path}")
    print(f"  Classes : {list(names.values())}")
    print(f"  Source  : {'screen' if is_screen else source}")
    print(f"  Conf    : {conf}   IoU: {iou}   Size: {imgsz}")
    print(f"\n  Q / ESC → quit  |  P → pause  |  S → screenshot\n")

    try:
        while True:
            if is_screen:
                raw   = sct.grab(monitor)           # grab tiap iterasi
                frame = np.array(raw)[:, :, :3]
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                offset = 250
                frame = frame[0+offset:cfg.FRAME_SIZE[1]+offset, 0+offset:cfg.FRAME_SIZE[0]+offset]  # crop to target size
                # frame = cv2.resize(frame, (cfg.FRAME_SIZE[0], cfg.FRAME_SIZE[1]), interpolation=cv2.INTER_AREA) #doing this changes the aspect ratio
                ret   = True
            else:
                ret, frame = cap.read()

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('p'):
                paused = not paused
            if key == ord('s'):
                fname = f"screenshot_{screenshot_idx:04d}.jpg"
                cv2.imwrite(fname, frame)
                print(f"Saved {fname}")
                screenshot_idx += 1

            if paused and last_results is not None:
                cv2.imshow("YOLO", frame)
                continue

            if not ret:
                print("  Stream ended or frame read failed.")
                break

            t0 = time.perf_counter()
            results = model.predict(
                source  = frame,
                conf    = conf,
                iou     = iou,
                imgsz   = imgsz,
                device  = device,
                verbose = False,
            )[0]
            t1 = time.perf_counter()
            fps_times.append(t1 - t0)
            fps = 1.0 / (sum(fps_times) / len(fps_times))

            best: dict[int, dict] = {}


            class_counts = {}
            if results.boxes is not None:
                for box in results.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf_val = float(box.conf[0])
                    cls_id   = int(box.cls[0])

                    if cls_id not in best or conf_val >= best[cls_id]['conf']:
                        color_id = color_det(x1, y1, x2, y2, frame, cls_id) 
                        color_str = cfg.ID_TO_COLOR.get(color_id, "unknown")
                        label = f"{names.get(cls_id, str(cls_id))}-{color_str}"

                        # draw_box(frame, x1, y1, x2, y2, label, conf_val, cls_id)
                        cx = (x1 + x2) / 2
                        cy = (y1 + y2) / 2
                        r  = max(x2 - x1, y2 - y1) / 2
                        cv2.circle(frame, (int(cx), int(cy)), int(r), color=(0, 255, 255), thickness=2)
                        cv2.drawMarker(frame, (int(cx), int(cy)), color=(0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)
                        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
                        best[cls_id] = {'x1': cx, 'y1': cy, 'r': r, 'conf': conf_val, 'color_id': color_id}
                        print(f"  Detected {label} {cls_id} {color_id} with conf {conf_val:.2f} at x {cx:.1f} y {cy:.1f} r {r:.1f}")

                    class_counts[cls_id] = class_counts.get(cls_id, 0) + 1

            data['ts'] = time.monotonic()

            for i in range(cfg.NUM_CLASSES):
                if i in best:
                    obj = best[i]
                    data['obj'][0][i]['x1']       = obj['x1']
                    data['obj'][0][i]['y1']       = obj['y1']
                    data['obj'][0][i]['r']        = obj['r']
                    data['obj'][0][i]['conf']     = obj['conf']
                    data['obj'][0][i]['class_id'] = i
                    data['obj'][0][i]['color_id'] = obj['color_id']
                    print(f" {obj['color_id']}")
                else:
                    data['obj'][0][i]['x1']       = -1
                    data['obj'][0][i]['y1']       = -1
                    data['obj'][0][i]['r']        = -1
                    data['obj'][0][i]['conf']     = -1
                    data['obj'][0][i]['class_id'] = i
                    data['obj'][0][i]['color_id'] = cfg.COLOR_DICT["unknown"]
            
            zmq_send(socket, data)


            det_count = sum(class_counts.values())
            draw_hud(frame, fps, det_count, paused, mname, frame_w, frame_h)
            draw_class_summary(frame, class_counts, names)
            last_results = results

            if gui:
                cv2.imshow("YOLO Live Inference", frame)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            zmq_close(socket)


        except FileNotFoundError:
            pass
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        print("Inference stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live YOLO inference")
    parser.add_argument("--source",  default="scr",        help="Camera index, video path, or RTSP URL")
    parser.add_argument("--model",   default=MODEL_PATH, help="Path to .pt weights file")
    parser.add_argument("--conf",    default=CONF_THRESH, type=float)
    parser.add_argument("--iou",     default=IOU_THRESH,  type=float)
    parser.add_argument("--imgsz",   default=IMG_SIZE,    type=int)
    parser.add_argument("--device",  default=DEVICE,      help="cpu / 0 / 0,1 / cuda:0")
    parser.add_argument("--gui", default=True, type=bool)
    args = parser.parse_args()

    run(
        source     = args.source,
        model_path = args.model,
        conf       = args.conf,
        iou        = args.iou,
        imgsz      = args.imgsz,
        device     = args.device,
        gui       = args.gui,
    )