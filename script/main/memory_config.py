from multiprocessing import shared_memory

import numpy as np, time

#every object is processed as circle

#there would be 4 classes, there would only be 4 objects passed to shared memory (guidebook didnt say there will be multiple object with the same class)

FRAME_SIZE = (644, 644, 3)


CLASS_DICT = {
    "square": 0,
    "hexagon": 1,
    "triangle": 2,
    "circle": 3
}

# Color lookup dictionary
COLOR_DICT = {
    "black":0, 
    "white":1, 
    "gray":2, 
    "red":3, 
    "orange":4, 
    "yellow":5, 
    "green":6, 
    "blue":7, 
    "purple":8, 
    "unknown":9
}

IPC_PATH = "ipc:///tmp/yolo.ipc"

# Inverse lookup for the inference script
ID_TO_COLOR = {v: k for k, v in COLOR_DICT.items()}
ID_TO_SHAPE = {v: k for k, v in CLASS_DICT.items()}

dtype = np.dtype([
    ('x1', np.float32),
    ('y1', np.float32),
    ('r', np.float32), #radius
    ('conf', np.float32),
    ('class_id', np.int16),
    ('color_id', np.int16)
])

NUM_CLASSES = len(CLASS_DICT)

meta_dtype = np.dtype([
    ('ts', np.float64),
    ('obj', dtype, NUM_CLASSES)
])

