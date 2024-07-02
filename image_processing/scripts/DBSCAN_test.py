import cv2
import numpy as np

# Load the depth image (assuming the image file is in the correct format)
depth_image = cv2.imread('image_processing/scripts/depth_000010.tiff', cv2.IMREAD_UNCHANGED)

# Ensure the image is of type 32FC1
if depth_image.dtype != np.float32:
    depth_image = depth_image.astype(np.float32)

print(depth_image.shape)
print(depth_image.dtype)

# Flatten the image and create a list of points (x, y, depth)
h, w = depth_image.shape
points = []

for i in range(h):
    for j in range(w):
        depth = depth_image[i, j]
        if depth > 0:  # Filter out zero depth points if needed
            points.append([i, j, depth])

points = np.array(points)

from sklearn.cluster import DBSCAN

# Define the DBSCAN parameters
eps = 5  # Maximum distance between two samples for them to be considered as in the same neighborhood
min_samples = 10  # Minimum number of samples in a neighborhood for a point to be considered a core point

# Fit DBSCAN to the data
dbscan = DBSCAN(eps=eps, min_samples=min_samples)
labels = dbscan.fit_predict(points)

# Add the labels to the points
points_with_labels = np.hstack((points, labels[:, np.newaxis]))

print(f"Number of clusters: {len(set(labels)) - (1 if -1 in labels else 0)}")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Plot the clusters
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for label in set(labels):
    if label == -1:
        # Noise points
        color = 'k'
    else:
        # Clusters
        color = plt.cm.jet(float(label) / max(labels + 1))

    cluster_points = points[labels == label]
    ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2], c=color, marker='o')

plt.show()
