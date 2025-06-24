'''
Project name: Semantic Segmentation for Adaptive Manipulation with Industrial Robots
Author: Pedro Martin
Electronics, Robotics and Mechatronics Engineering - University of Malaga
Date: 2025
 
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Generate points on a sphere
np.random.seed(0)
n_points = 1000
phi = np.random.uniform(0, np.pi, n_points)          # Polar angle
theta = np.random.uniform(0, 2 * np.pi, n_points)    # Azimuthal angle
r = 1.0                                              # Sphere radius

x = r * np.sin(phi) * np.cos(theta)
y = r * np.sin(phi) * np.sin(theta)
z = r * np.cos(phi)

points_sphere = np.vstack((x, y, z)).T               # Shape (n_points, 3)

# Select a reference point and find nearby points within a threshold distance
reference_point =  np.array([0.7, 0, 0.7]) / np.linalg.norm([0.7, 0, 0.7])
distances = np.linalg.norm(points_sphere - reference_point, axis=1)
mask = distances < 0.1                               # Threshold radius for local neighborhood
local_points = points_sphere[mask]

# Compute the centroid of the local points
centroid = np.mean(local_points, axis=0)

# Center the local points by subtracting the centroid
points_centered = local_points - centroid

# Perform Singular Value Decomposition (SVD) on centered points
# vh contains the right singular vectors as rows
_, _, vh = np.linalg.svd(points_centered)

# The normal vector to the best-fit plane is the singular vector
# corresponding to the smallest singular value (last row of vh)
normal_vector = vh[-1]

# Create a grid of points for the tangent plane around the centroid
size = 0.2
xx, yy = np.meshgrid(np.linspace(-size, size, 10), np.linspace(-size, size, 10))

# Calculate corresponding z values on the plane using the plane equation
zz = (-normal_vector[0] * xx - normal_vector[1] * yy) / normal_vector[2]

# Shift the plane grid to the centroid location
plane_x = xx + centroid[0]
plane_y = yy + centroid[1]
plane_z = zz + centroid[2]

# Plotting
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot all points on the sphere (faint)
ax.scatter(points_sphere[:, 0], points_sphere[:, 1], points_sphere[:, 2], alpha=0.2, label='Entire sphere point cloud')

# Plot the local neighborhood points (blue)
ax.scatter(local_points[:, 0], local_points[:, 1], local_points[:, 2], color='blue', alpha=0.5, label='Local neighborhood')

# Plot the estimated tangent plane (semi-transparent cyan)
ax.plot_surface(plane_x, plane_y, plane_z, alpha=0.4, color='cyan')

# Mark the centroid (red)
ax.scatter(*centroid, color='red', s=100, label='Local centroid')

# Draw the normal vector at the centroid (black arrow)
ax.quiver(*centroid, *normal_vector, color='black', length=0.2, normalize=True, label='Estimated normal vector')

# Labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Tangent plane estimation on a spherical surface using SVD (np.linalg.svd)')

ax.legend(loc='upper left')
plt.tight_layout()
plt.show()
