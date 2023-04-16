from dataclasses import *
from typing import *
from collections import *
import numpy as np
import random
import time
import tkinter as tk




FRAME_SCALE = (2, 800) # m, px
FRAME_BUFFER = np.zeros((FRAME_SCALE[1], FRAME_SCALE[1]))
root = tk.Tk()
canvas = tk.Canvas(root, width=FRAME_SCALE[1], height=FRAME_SCALE[1], borderwidth=0, highlightthickness=0, bg="black")
canvas.grid()
def create_circle(x, y, r, c, color, fill): #center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    if fill:
        return c.create_oval(x0, y0, x1, y1, outline="#%02x%02x%02x" % color, fill="#%02x%02x%02x" % color)
    else:
        return c.create_oval(x0, y0, x1, y1, outline="#%02x%02x%02x" % color)

# canvas.update()
"""
WORLD COORDINATE SYSTEM

^
Y  *
|
-----X->

"""


def display():
    global FRAME_BUFFER, FRAME_SCALE, root
    root.update()


def clean():
    global FRAME_BUFFER, FRAME_SCALE, canvas, root
    FRAME_BUFFER = np.zeros((FRAME_SCALE[1], FRAME_SCALE[1]))
    canvas.delete("all")
    root.update()


def world_to_image_d(d: float) -> int:
    global FRAME_BUFFER, FRAME_SCALE
    return int(round(d*FRAME_SCALE[1]/FRAME_SCALE[0]))


def world_to_image_point(point: np.ndarray) -> tuple:
    global FRAME_BUFFER, FRAME_SCALE
    x_w = point[0]
    y_w = point[1]
    x_img = world_to_image_d(x_w) + FRAME_SCALE[1]/2
    y_img = world_to_image_d(-y_w) + FRAME_SCALE[1]/2
    return int(x_img), int(y_img)


def draw_circle_px_px(x: int, y:int, radius: int, color: tuple, fill=False):
    global FRAME_BUFFER, FRAME_SCALE, canvas
    create_circle(x, y, radius, canvas, color, fill)


def draw_circle_m_m(center: np.ndarray, radius: float, color: tuple, fill=False):
    global FRAME_BUFFER, FRAME_SCALE
    x, y = world_to_image_point(center)
    draw_circle_px_px(x, y, world_to_image_d(radius), color, fill)


def solve_position(b_positions: np.ndarray, b_distances: np.ndarray) -> np.ndarray:
    b_p = b_positions[1:, :]
    b_p_1 = b_positions[0]
    # print(b_p_1, b_p)
    A = (b_p_1 - b_p)
    # print("A", A)
    # print("b_p**2", np.expand_dims(np.sum(-b_p**2, axis=1), axis=1), np.sum(b_p_1**2), b_distances[0]**2, b_distances[1:]**2, np.expand_dims(b_distances[1:]**2, axis=1))
    b = np.expand_dims(np.sum(-b_p**2, axis=1), axis=1) + np.sum(b_p_1**2) - b_distances[0]**2 + np.expand_dims(b_distances[1:]**2, axis=1)
    b = b*0.5
    # print("b", b)
    # print("------", A.T @ A)
    m = np.linalg.inv(A.T @ A) @ (A.T @ b)
    # print(m)
    return np.reshape(m, (2))

def rssi_to_distance(rssi: float, m_power: float, a: float, b: float, c: float) -> float:  # rssi, m_power -> distance
    return a * ((rssi / m_power) ** b) + c


def distance_to_rssi(distance: float, m_power: float, a: float, b: float, c: float) -> float:  # distance, m_power -> rssi
    return m_power * ((distance - c) / a) ** (1 / b)


prev_noises: Dict[str, List[float]] = dict()


def apply_noise_to_rssi(rssi: float, b: str, distance: float) -> float:
    global prev_noises
    if b not in prev_noises.keys():
        prev_noises[b] = []
    noise_c = (random.random() * 2 - 1) * 0.01 * rssi
    # if distance > 0.4:
    #     noise_c = noise_c * np.clip(1.2, 0.1, (distance/1)*10000)
    prev_noises[b] = [noise_c] + prev_noises[b][:3 - 1]
    noise = sum(prev_noises[b]) / len(prev_noises[b])
    return rssi + noise


def apply_noise_to_rssi_2(rssi: float, b: str, distance: float) -> float:
    return np.random.normal(rssi, 0.2)


def calc_beacon_rssi(b_position: np.ndarray, m_power: float, a: float, b: float, c: float, r_position: np.ndarray, b_id: str) -> float:
    #
    # print("calc_beacon_rssi")
    distance = np.sum((b_position - r_position)**2)**0.5
    # print(" distance: ", distance)
    rssi = distance_to_rssi(distance, m_power, a, b, c)
    rssi = apply_noise_to_rssi_2(rssi, b_id, distance) # some noise,
    return rssi


beacons_poses: Dict[str, np.ndarray] = dict()

beacons_poses["1"] = np.array([0.1, 0.1])
beacons_poses["2"] = np.array([0.1, 0.5])
beacons_poses["3"] = np.array([0.5, 0.1])
beacons_poses["4"] = np.array([-0.3, -0.1])

robot_poses: Dict[str, np.ndarray] = dict()
robot_yaws: Dict[str, float] = dict()
robot_cmds: Dict[str, np.ndarray] = dict()

robot_poses["r1"] = np.array([-0.2, 0.2])
robot_cmds["r1"] = np.array([0, 0]) # [v, w]

r_pose_pr: Dict[str, np.ndarray] = dict()


rb_rssi_val: Dict[str, Dict[str, float]] = dict()
rb_dist: Dict[str, Dict[str, float]] = dict()
rb_dist_f: Dict[str, Dict[str, float]] = dict()
rb_dist_hist: Dict[str, Dict[str, List[float]]] = dict()
rb_dist_h_size: int = 2

FRQ = 20
m_power = -50
k_a = 1
k_b = 10
k_c = 0

t_start = time.time()





while True:
    time_stamp = (time.time() - t_start)
    # Draw beacons and receivers
    for b in beacons_poses:
        p = beacons_poses[b]
        draw_circle_m_m(p, 0.01, (0, 200, 200), fill=True)
    # display()
    for b in robot_poses:
        p = robot_poses[b]
        draw_circle_m_m(p, 0.01, (200, 0, 0), fill=True)
    # display()

    # Calc rssi for receivers
    for r in robot_poses:
        rp = robot_poses[r]
        beacons_rssi_vals = {i: calc_beacon_rssi(beacons_poses[i], m_power, k_a, k_b, k_c, rp, i) for i in beacons_poses}
        rb_rssi_val[r] = beacons_rssi_vals

    print("rb_rssi_val", rb_rssi_val)

    # Predict distance
    for r in robot_poses:
        rssi_vals = rb_rssi_val[r]
        if r not in rb_dist.keys():
            rb_dist[r] = dict()
        for b in rssi_vals:
            rb_dist[r].update({b: rssi_to_distance(rssi_vals[b], m_power, k_a, k_b, k_c)})

    # print("rb_dist", rb_dist)

    # Draw simple distances
    # for r in rb_dist:
    #     for b in rb_dist[r]:
    #         d = rb_dist[r][b]
    #         draw_circle_m_m(beacons_poses[b], d, (0, 50, 200))
            # draw_circle_m_m(beacons_poses[b], np.sum((beacons_poses[b] - receivers_poses[r])**2)**0.5, (200, 0, 0))
    # display()

    # filter distance
    for r in robot_poses:
        if r not in rb_dist_f.keys():
            rb_dist_f[r] = dict()
            rb_dist_hist[r] = dict()
        for b in rb_dist[r]:
            # r b
            if b not in rb_dist_hist[r].keys():
                rb_dist_hist[r].update({b: []})
            # rb_dist_hist[r][b].append(b)
            rb_dist_hist[r][b] = [rb_dist[r][b]] + rb_dist_hist[r][b][:rb_dist_h_size-1]
            # print("len", len(rb_dist_hist[r][b]))
            mv = sum(rb_dist_hist[r][b]) / len(rb_dist_hist[r][b])
            rb_dist_f[r][b] = mv*0.9 + rb_dist[r][b]*0.1
    # print("rb_dist_hist", rb_dist_hist)
    # print("rb_dist_f", rb_dist_f)

    # Draw filtered distances
    for r in rb_dist_f:
        for b in rb_dist_f[r]:
            d = rb_dist_f[r][b]
            # draw_circle_m_m(beacons_poses[b], d, (0, 200, 0))
    # display()

    # solve_position
    for r in robot_poses:
        r_pose_pr[r] = solve_position(np.array([beacons_poses[i] for i in beacons_poses])[:, :2], np.array([rb_dist_f[r][i] for i in rb_dist_f[r]]))
    # print("r_pose_pr", r_pose_pr)

    for r in r_pose_pr:
        p = r_pose_pr[r]
        draw_circle_m_m(p, 0.01, (0, 0, 255), fill=True)

    print(np.sum((robot_poses["r1"][:2] - r_pose_pr["r1"][:2]) ** 2) ** 0.5)


    # Move robots
    # for r in robot_poses:
    #     robot_poses[r] = (np.array([[np.cos(robot_poses[r][2]), 0],
    #                                 [np.sin(robot_poses[r][2]), 0],
    #                                 [0, 1]]) @ robot_cmds[r].T).T * (1/FRQ) + robot_poses[r]

    robot_poses["r1"] = np.array([np.cos(time_stamp), np.sin(time_stamp)]) * 0.28 + np.array([0, 0.3])
    beacons_poses["1"] = np.array([np.cos(-time_stamp), np.sin(-time_stamp)]) * 0.28 + np.array([0, -0.3])

    display()
    time.sleep(1/FRQ)
    # while True: root.update()
    clean()







