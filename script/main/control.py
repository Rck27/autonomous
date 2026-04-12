from multiprocessing import shared_memory
import numpy as np, time
import memory_config as cfg
import cv2 as cv

import zmq, msgpack_numpy as m

img = np.zeros((644, 644, 3), np.uint8)

STALE_THRESHOLD = 0.02  # 50ms
ALIVE_THRESHOLD = 1.0
last_received = time.monotonic()


data = np.zeros((1, cfg.NUM_CLASSES), dtype=cfg.meta_dtype)

def zmq_connect():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(cfg.IPC_PATH)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.RCVTIMEO, 20)  # Set receive timeout to 100ms
    return socket

def zmq_receive(socket):
    try:
        msg = socket.recv(flags=zmq.NOBLOCK)
        data = m.unpackb(msg, raw=False)
        return data
    except zmq.Again:
        return None

def zmq_close(socket):
    socket.close()

try:
    socket = zmq_connect()
    while True:
        data = zmq_receive(socket)
        now = time.monotonic()
        if data is not None:
            last_received = now
            timestamp = data['ts'][0]
            print(f" delay {time.monotonic() - timestamp:.2f} seconds")
            cv.drawMarker(img, (int(644/2), int(644/2)), color=(255, 255, 255), markerType=cv.MARKER_TILTED_CROSS, markerSize=20, thickness=1)
            if time.monotonic() - timestamp > STALE_THRESHOLD:
                print(f"Data is stale: ")
            else:
                print(f" objects detected at {timestamp:.2f}")
                for i in range(cfg.NUM_CLASSES):
                    obj = data['obj'][0][i]
                    if obj['conf'] > 0:
                        cv.drawMarker(img, (int(obj['x1']), int(obj['y1'])), color=(0, 255, 255), markerType=cv.MARKER_CROSS, markerSize=10, thickness=1)
                        print(f"  class_id {obj['class_id']} {cfg.ID_TO_SHAPE[obj['class_id']]} color_id {obj['color_id']} {cfg.ID_TO_COLOR[obj['color_id']]} \n conf {obj['conf']:.2f} x1 {obj['x1']:.1f} y1 {obj['y1']:.1f} r {obj['r']:.1f}")
                cv.imshow("Live Inference", img)
                img *= 0
        else:
            if now - last_received > ALIVE_THRESHOLD:
                img *= 0
                cv.imshow("Live Inference", img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("\nInterrupted by user.")
    zmq_close(socket)