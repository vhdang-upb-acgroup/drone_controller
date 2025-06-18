import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def get_trajectory_point(t, type="figure8", z0=5.0):
    t = np.asarray(t)
    omega = 0.5

    if type == "figure8":
        A, B = 2.0, 1.0
        x = A * np.sin(omega * t)
        y = B * np.sin(omega * t) * np.cos(omega * t)
        z = np.full_like(t, z0)
        return x, y, z

    elif type == "circle":
        radius = 2.0
        x = radius * np.cos(omega * t)
        y = radius * np.sin(omega * t)
        z = np.full_like(t, z0)
        return x, y, z

    else:
        x = np.zeros_like(t)
        y = np.zeros_like(t)
        z = np.full_like(t, z0)
        return x, y, z

# --- Animation Section ---

# Time values
t_vals = np.linspace(0, 18, 1000)

# Choose trajectory type here:
trajectory_type = "figure8"  # Try "circle", "lemniscate", "square", etc.
x_vals, y_vals, z_vals = get_trajectory_point(t_vals, type=trajectory_type)

# Plot setup
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_vals, y_vals, z_vals, color='blue', linewidth=1, label='Trajectory')

point_mass, = ax.plot([], [], [], 'ro', markersize=18, label='Point Mass')

# Axis limits
ax.set_xlim(np.min(x_vals)-2, np.max(x_vals)+2)
ax.set_ylim(np.min(y_vals)-2, np.max(y_vals)+2)
ax.set_zlim(np.min(z_vals)-2, np.max(z_vals)+2)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title(f'Animated Point Mass Along "{trajectory_type}" Trajectory')
ax.legend()

def update(frame):
    point_mass.set_data([x_vals[frame]], [y_vals[frame]])
    point_mass.set_3d_properties([z_vals[frame]])
    return point_mass,

ani = FuncAnimation(fig, update, frames=len(t_vals), interval=20, blit=True)

plt.tight_layout()
plt.show()
