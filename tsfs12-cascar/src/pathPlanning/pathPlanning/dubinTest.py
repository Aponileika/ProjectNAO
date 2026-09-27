import json
import numpy as np
import matplotlib.pyplot as plt

def add_rectangle_obstacle(grid, obstacle, xlim, ylim, resolution):
    """Add a rectangular obstacle defined in world coordinates to the occupancy grid."""
    x_min, x_max, y_min, y_max = obstacle

    x0 = int((x_min - xlim[0]) / resolution)
    x1 = int((x_max - xlim[0]) / resolution)
    y0 = int((y_min - ylim[0]) / resolution)
    y1 = int((y_max - ylim[0]) / resolution)

    x0 = max(0, x0)
    x1 = min(grid.shape[1], x1)
    y0 = max(0, y0)
    y1 = min(grid.shape[0], y1)

    grid[y0:y1, x0:x1] = 1
    return grid

xlim = (-5,5)
ylim = xlim
xy_resolution = 0.05

size = int((xlim[1]-xlim[0])/xy_resolution)
map = np.zeros((size,size))

obstacles = [
    (-0.5, 0.5, -3, 3),
    (-3, -2, -3, -2),
    (2,3,2,3),
    (-3,-2,2,3),
    (2,3,-3,-2)
]

with open("result.json") as f:
    data = json.load(f)

start = np.array(data["start"])
goal = np.array(data["goal"])
path = np.array(data["path"])

fig, ax = plt.subplots()

for x_min, x_max, y_min, y_max in obstacles:
    ax.fill(
        [x_min, x_max, x_max, x_min],
        [y_min, y_min, y_max, y_max],
        alpha=0.5
    )

# Path
ax.plot(path[:, 0], path[:, 1], "-o", markersize=3)

# Heading arrows
ax.quiver(start[0], start[1], np.cos(start[2]), np.sin(start[2]),
            angles="xy", scale_units="xy", scale=1, color="green", label="Start" )

ax.quiver(goal[0], goal[1], np.cos(goal[2]), np.sin(goal[2]),
            angles="xy", scale_units="xy", scale=1, color="red", label="Goal")

# Make x/y have identical physical scale
ax.set_aspect("equal", adjustable="box")

# Determine plot limits from the data
all_x = np.concatenate([path[:, 0], [start[0], goal[0]]])
all_y = np.concatenate([path[:, 1], [start[1], goal[1]]])

xmin, xmax = all_x.min(), all_x.max()
ymin, ymax = all_y.min(), all_y.max()
xlim = [-5,5]
ylim = [-5,5]

ax.set_xlim(xlim)
ax.set_ylim(ylim)
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.grid()
ax.legend()

ax.grid()
ax.legend()

plt.show()
