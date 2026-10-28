#!/usr/bin/env python3
"""Generate a labelled synthetic dataset from the Gazebo overhead camera.

For each sample: move the three objects to random poses, grab a camera frame,
compute exact YOLO labels by projecting each object's 3D shape into the image,
and apply lighting/noise variation to the saved image.

Needs the world and the /camera/image bridge running. No robot required.
"""
import math
import random
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

WORLD = "pickplace"
OUT = Path.home() / "pickplace_ws" / "dataset"
N_IMAGES = 300
VAL_COUNT = 60

# Overhead camera: must match pickplace.sdf
CAM_X, CAM_Y, CAM_Z = 0.6, 0.0, 0.8
IMG_W, IMG_H = 320, 240
HFOV = 1.05
FOCAL = (IMG_W / 2) / math.tan(HFOV / 2)

X_RANGE = (0.38, 0.82)
Y_RANGE = (-0.12, 0.32)
MIN_DIST = 0.10

# Starting positions from pickplace.sdf
START_XY = [(0.5, 0.1), (0.6, -0.1), (0.7, 0.15)]

CLASS_NAMES = ["cube", "cylinder", "box"]


def box_pts(sx, sy, sz):
    return [(x * sx / 2, y * sy / 2, z * sz / 2)
            for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]


def cyl_pts(r, h, n=16):
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        for z in (-1, 1):
            pts.append((r * math.cos(a), r * math.sin(a), z * h / 2))
    return pts


# model name, class id, local shape points, centre height above table
OBJECTS = [
    ("cube_red", 0, box_pts(0.05, 0.05, 0.05), 0.025),
    ("cylinder_green", 1, cyl_pts(0.025, 0.08), 0.04),
    ("box_blue", 2, box_pts(0.06, 0.04, 0.05), 0.025),
]


def sample_poses(prev_xy):
    """Random poses that don't overlap each other or the old positions."""
    while True:
        poses = [(random.uniform(*X_RANGE), random.uniform(*Y_RANGE),
                  random.uniform(0, 2 * math.pi)) for _ in OBJECTS]
        new_xy = [p[:2] for p in poses]
        ok = True
        for i in range(3):
            for j in range(3):
                if i < j and math.dist(new_xy[i], new_xy[j]) < MIN_DIST:
                    ok = False
                if i != j and math.dist(new_xy[i], prev_xy[j]) < MIN_DIST:
                    ok = False
        if ok:
            return poses


def set_pose(name, x, y, z, yaw, retries=4):
    req = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.5f}, '
           f'w: {math.cos(yaw / 2):.5f}}}')
    last = ""
    for _ in range(retries):
        res = subprocess.run(
            ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "5000", "--req", req],
            capture_output=True, text=True)
        if "true" in res.stdout.lower():
            return
        last = f"{res.stdout} {res.stderr}"
        time.sleep(1.0)
    raise RuntimeError(f"set_pose failed for {name} after {retries} tries: {last}")


def project(pts, x, y, z, yaw):
    """Pixel bounding box of an object. Image up = world +x, image left = world +y."""
    c, s = math.cos(yaw), math.sin(yaw)
    us, vs = [], []
    for px, py, pz in pts:
        wx = x + c * px - s * py
        wy = y + s * px + c * py
        wz = z + pz
        depth = CAM_Z - wz
        us.append(IMG_W / 2 - FOCAL * (wy - CAM_Y) / depth)
        vs.append(IMG_H / 2 - FOCAL * (wx - CAM_X) / depth)
    return min(us), min(vs), max(us), max(vs)


def augment(img):
    """Image-space lighting, colour and noise variation."""
    img = img.astype(np.float32)
    img = img * random.uniform(0.6, 1.3) + random.uniform(-25, 25)
    img *= np.array([random.uniform(0.9, 1.1) for _ in range(3)], np.float32)
    img += np.random.normal(0, random.uniform(0, 6), img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    if random.random() < 0.3:
        img = cv2.GaussianBlur(img, (3, 3), 0)
    return img


def draw_debug(img, boxes):
    out = img.copy()
    for cls, u0, v0, u1, v1 in boxes:
        cv2.rectangle(out, (int(u0), int(v0)), (int(u1), int(v1)), (0, 255, 255), 1)
        cv2.putText(out, CLASS_NAMES[cls], (int(u0), max(int(v0) - 3, 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    return cv2.resize(out, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)


class Grabber(Node):
    def __init__(self):
        super().__init__("dataset_grabber")
        self.latest = None
        self.create_subscription(Image, "/camera/image", self.cb, 1)

    def cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        self.latest = cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)

    def fresh_frame(self, settle=0.8):
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.latest = None
        t0 = time.time()
        while self.latest is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 5:
                raise RuntimeError("No camera image. Is the bridge running?")
        return self.latest.copy()


def main():
    random.seed(0)
    np.random.seed(0)
    for split in ("train", "val"):
        (OUT / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT / "labels" / split).mkdir(parents=True, exist_ok=True)
        (OUT / "debug").mkdir(exist_ok=True)
    start = len(list((OUT / "images").glob("*/img_*.png")))
    random.seed(start)
    np.random.seed(start)
    print(f"Resuming from image {start}")

    rclpy.init()
    grab = Grabber()
    prev_xy = list(START_XY)

    for i in range(start,N_IMAGES):
        split = "val" if i < VAL_COUNT else "train"
        poses = sample_poses(prev_xy)
        lines, boxes = [], []
        for (name, cls, pts, zc), (x, y, yaw) in zip(OBJECTS, poses):
            set_pose(name, x, y, zc + 0.002, yaw)
            u0, v0, u1, v1 = project(pts, x, y, zc, yaw)
            u0, u1 = max(u0, 0), min(u1, IMG_W)
            v0, v1 = max(v0, 0), min(v1, IMG_H)
            cx, cy = (u0 + u1) / 2 / IMG_W, (v0 + v1) / 2 / IMG_H
            w, h = (u1 - u0) / IMG_W, (v1 - v0) / IMG_H
            lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
            boxes.append((cls, u0, v0, u1, v1))
        prev_xy = [p[:2] for p in poses]

        frame = augment(grab.fresh_frame())
        stem = f"img_{i:04d}"
        cv2.imwrite(str(OUT / "images" / split / f"{stem}.png"), frame)
        (OUT / "labels" / split / f"{stem}.txt").write_text("\n".join(lines) + "\n")
        if i < 5:
            cv2.imwrite(str(OUT / "debug" / f"{stem}_boxes.png"), draw_debug(frame, boxes))
        if (i + 1) % 20 == 0:
            print(f"{i + 1}/{N_IMAGES} images")

    (OUT / "data.yaml").write_text(
        "path: /content/dataset\ntrain: images/train\nval: images/val\n"
        "names:\n  0: cube\n  1: cylinder\n  2: box\n")
    print(f"Done. Dataset in {OUT}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()