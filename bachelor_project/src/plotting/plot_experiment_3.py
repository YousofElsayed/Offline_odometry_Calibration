#!/usr/bin/env python3
import os
import matplotlib.pyplot as plt
import csv

data_dir = '/home/yousof/catkin_ws/src/bachelor_project/results/final_pose_error'

methods = ["_bostani", "_a_umb", "_rot"]
method_names = {"_bostani": "Bostani Method", "_a_umb": "A-UMB Method", "_rot": "Tomasi Method"}
colors = {"_bostani": "g", "_a_umb": "b","_rot": "r" }
markers = { "_bostani": "^", "_a_umb": "s","_rot": "o"}

def load_traj(filename):
    xs, ys = [], []
    if not os.path.exists(filename):
        return xs, ys
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return xs, ys

def load_endpoint(filename):
    if not os.path.exists(filename):
        return None
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        points = [row for row in reader]
        if points:
            x = float(points[-1]['x'])
            y = float(points[-1]['y'])
            return (x, y)
    return None

# --- PLOT 1: Circle ---
plt.figure(figsize=(10, 8))
title = "Circle, d = 5 m"
for m in methods:
    xs, ys = load_traj(os.path.join(data_dir, f"circle_traj{m}.csv"))
    endpoint = load_endpoint(os.path.join(data_dir, f"circle_endpoint{m}.csv"))
    label = method_names[m]
    if xs and ys:
        plt.plot(xs, ys, color=colors[m], label=label)
    if endpoint is not None:
        plt.plot(endpoint[0], endpoint[1], marker=markers[m], color=colors[m], linestyle='--', markersize=10, label=f"{label} Endpoint")
plt.title(title)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.grid(True)
plt.axis('equal')
plt.legend(loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(data_dir, 'circle_comparison.png'), dpi=1200)
plt.close()

# --- PLOT 2: Triangle ---
plt.figure(figsize=(10, 8))
title = "Triangle, side = 5 m"
for m in methods:
    xs, ys = load_traj(os.path.join(data_dir, f"triangle_traj{m}.csv"))
    endpoint = load_endpoint(os.path.join(data_dir, f"triangle_endpoint{m}.csv"))
    label = method_names[m]
    if xs and ys:
        plt.plot(xs, ys, color=colors[m], label=label)
    if endpoint is not None:
        plt.plot(endpoint[0], endpoint[1], marker=markers[m], color=colors[m], linestyle='', markersize=10, label=f"{label} Endpoint")
plt.title(title)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.grid(True)
plt.axis('equal')
plt.legend(loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(data_dir, 'triangle_comparison.png'), dpi=1200)
plt.close()

# --- PLOT 3: Rectangle ---
plt.figure(figsize=(10, 8))
title = "Rectangle, 10x5 m"
for m in methods:
    xs, ys = load_traj(os.path.join(data_dir, f"rectangle_traj{m}.csv"))
    endpoint = load_endpoint(os.path.join(data_dir, f"rectangle_endpoint{m}.csv"))
    label = method_names[m]
    if xs and ys:
        plt.plot(xs, ys, color=colors[m], label=label)
    if endpoint is not None:
        plt.plot(endpoint[0], endpoint[1], marker=markers[m], color=colors[m], linestyle='', markersize=10, label=f"{label} Endpoint")
plt.title(title)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.grid(True)
plt.axis('equal')
plt.legend(loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(data_dir, 'rectangle_comparison.png'), dpi=1200)
plt.close()

# --- PLOT 4: Figure Eight ---
plt.figure(figsize=(10, 8))
title = "Figure Eight, d = 5 m"
for m in methods:
    xs, ys = load_traj(os.path.join(data_dir, f"figure_eight_traj{m}.csv"))
    endpoint = load_endpoint(os.path.join(data_dir, f"figure_eight_endpoint{m}.csv"))
    label = method_names[m]
    if xs and ys:
        plt.plot(xs, ys, color=colors[m], label=label)
    if endpoint is not None:
        plt.plot(endpoint[0], endpoint[1], marker=markers[m], color=colors[m], linestyle='', markersize=10, label=f"{label} Endpoint")
plt.title(title)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.grid(True)
plt.axis('equal')
plt.legend(loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(data_dir, 'figure_eight_comparison.png'), dpi=1200)
plt.close()

print('All 4 plots saved as PNGs in', data_dir)
