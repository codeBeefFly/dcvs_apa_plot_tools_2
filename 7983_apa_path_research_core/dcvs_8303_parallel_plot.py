import numpy as np
import math
import matplotlib.pyplot as plt

#computed_path = "width6/steering_53/computed_path.txt"
computed_path = "car_width_19/width_55/steering_45/computed_path.txt"
half_car_width = 0.5 * 1.935
parking_width = 5.5

def get_affine(pose):
    theta = pose[2]
    return np.asarray([[np.cos(theta), -np.sin(theta), pose[0]],
                       [np.sin(theta), np.cos(theta), pose[1]]])

def getPath(pt, path):
    pt_path = np.zeros((path.shape[0], 2))
    for idx in range(path.shape[0]):
        pose = path[idx, :]
        mat = get_affine(pose)
        trans_pt = np.matmul(mat, pt)
        pt_path[idx, :] = trans_pt
    return pt_path

path = []
try:
    with (open(computed_path,"r")) as f:
        for line in f:
            if ("x y" not in line):
                pose = []
                pose.append(float(line.split()[0]))
                pose.append(float(line.split()[1]))
                pose.append(float(line.split()[2]))
                path.append(pose)
except IOError:
    print ("Could not find golden computed_path file!")
    
path = np.asarray(path)
print("path shape {}".format(path.shape))

ptA = np.asarray([0.926 + 2.92, half_car_width, 1])
ptB = np.asarray([0.926 + 2.92, -half_car_width, 1])
ptC = np.asarray([-0.94, -half_car_width, 1])
ptD = np.asarray([-0.94, half_car_width, 1])
print("pt A = {}".format(ptA))
print("pt B = {}".format(ptB))
print("pt C = {}".format(ptC))
print("pt D = {}".format(ptD))

pathA = getPath(ptA, path)
pathB = getPath(ptB, path)
pathC = getPath(ptC, path)
pathD = getPath(ptD, path)


alpha = 0.3
def draw_rectangle(ax, pa, pb, pc, pd, pt, c = 'r', alpha = alpha, label = None):
    ax.plot(pt[0], pt[1], c + 'o', alpha = alpha)
    if label is None:
        ax.plot([pa[0], pb[0]], [pa[1], pb[1]], c, alpha = alpha)
        ax.plot([pb[0], pc[0]], [pb[1], pc[1]], c, alpha = alpha)
        ax.plot([pc[0], pd[0]], [pc[1], pd[1]], c, alpha = alpha)
        ax.plot([pd[0], pa[0]], [pd[1], pa[1]], c, alpha = alpha)
    else:
        ax.plot([pa[0], pb[0]], [pa[1], pb[1]], c, alpha = alpha, label = label)
        ax.plot([pb[0], pc[0]], [pb[1], pc[1]], c, alpha = alpha)
        ax.plot([pc[0], pd[0]], [pc[1], pd[1]], c, alpha = alpha)
        ax.plot([pd[0], pa[0]], [pd[1], pa[1]], c, alpha = alpha)
        


def draw_parkingspot(ax, enlarge_ratio, c = 'm'):
    center = np.asarray([4.77, 3.43478])
    half_length = 0.5 * 2.47746
    half_width = 0.5 * parking_width * enlarge_ratio
    pa = center + np.asarray([-half_width, -half_length])
    pb = center + np.asarray([half_width, -half_length])
    pc = center + np.asarray([half_width, half_length])
    pd = center + np.asarray([-half_width, half_length])
    ax.plot([pa[0], pd[0]], [pa[1], pd[1]], c, linewidth = 2)
    ax.plot([pd[0], pc[0]], [pd[1], pc[1]], c, linewidth = 2, label = "parking spot")
    ax.plot([pc[0], pb[0]], [pc[1], pb[1]], c, linewidth = 2)
    width_string = "{:.2f}m".format(2 * half_width)
    length_string = "{:.2f}m".format(2 * half_length)
    ax.text(pd[0] + half_width, pd[1], width_string, fontsize = 12)
    ax.text(pb[0], pb[1] + half_length, length_string, fontsize = 12)

fig, ax = plt.subplots()
ax.plot(path[:, 0], path[:, 1], 'b-', linewidth=2, label = "computed_path")

num_skips = 2
for idx in range(path.shape[0]):
    if idx % num_skips == 0:
        draw_rectangle(ax, pathA[idx, :], pathB[idx, :], pathC[idx, :], pathD[idx, :], path[idx, :2])

draw_rectangle(ax, pathA[-1, :], pathB[-1, :], pathC[-1, :], pathD[-1, :], path[-1, :2], 'g', 1, "final car bbox")
draw_parkingspot(ax, 1.0)
ax.legend()
ax.set_title("max_steering = 0.45")
ax.set_xlabel("x(m)")
ax.set_ylabel("y(m)")

plt.show()


