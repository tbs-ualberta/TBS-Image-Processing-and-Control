import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------- Camera calibration constants -----------------------------------------------
# Intrinsic parameters
DEPTH_INTRINSICS = np.array([[366.193, 0, 256.684],
                             [0, 366.193, 207.085],
                             [0, 0, 1]])
DEPTH_DIST_COEFFS = np.array([0.0893804, -0.272566, 0, 0, 0.0958438])

RGB_INTRINSICS = np.array([[1081.37, 0, 959.5],
                           [0, 1081.37, 539.5],
                           [0, 0, 1]])

R = np.eye(3)  # Rotation matrix (3x3)
T = np.array([0, 0, 0])  # Translation vector (3x1)

# --------------------------------------------------------------------------------------------------------------------


# -------------------------------------------- Calculated constants --------------------------------------------------

EXTRINSIC_MATRIX = np.eye(4)
EXTRINSIC_MATRIX[:3, :3] = R
EXTRINSIC_MATRIX[:3, 3] = T

# --------------------------------------------------------------------------------------------------------------------


# --------------------------------------------- For image masking ----------------------------------------------------
class ProcessingResults:
    def __init__(self, image, masks, boxes, phrases, logits):
        # Constructor
        self.image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        self.masks = masks
        # Convert masks to numpy arrays
        self.masks_np = [mask.squeeze().cpu().numpy() for mask in masks]

        self.boxes = boxes
        self.phrases = phrases
        self.logits = logits

    def display_image_with_masks(self):
        num_masks = len(self.masks)

        fig, axes = plt.subplots(1, num_masks + 1, figsize=(15, 5))
        axes[0].imshow(self.image)
        axes[0].set_title("Original Image")
        axes[0].axis('off')

        for i, mask_np in enumerate(self.masks):
            axes[i+1].imshow(mask_np, cmap='gray')
            axes[i+1].set_title(f"Mask {i+1}")
            axes[i+1].axis('off')

        plt.tight_layout()
        plt.show()

    def display_image_with_boxes(self):
        fig, ax = plt.subplots()
        ax.imshow(self.image)
        ax.set_title("Image with Bounding Boxes")
        ax.axis('off')

        for box, logit in zip(self.boxes, self.logits):
            x_min, y_min, x_max, y_max = box
            confidence_score = round(logit.item(), 2)  # Convert logit to a scalar before rounding
            box_width = x_max - x_min
            box_height = y_max - y_min

            # Draw bounding box
            rect = plt.Rectangle((x_min, y_min), box_width, box_height, fill=False, edgecolor='red', linewidth=2)
            ax.add_patch(rect)

            # Add confidence score as text
            ax.text(x_min, y_min, f"Confidence: {confidence_score}", fontsize=8, color='red', verticalalignment='top')

        plt.show()

# --------------------------------------------------------------------------------------------------------------------

# ---------------------------------------------- Print functions -----------------------------------------------------

    def print_bounding_boxes(self):
        print("Bounding Boxes:")
        for i, box in enumerate(self.boxes):
            print(f"Box {i+1}: {box}")

    def print_detected_phrases(self):
        print("\nDetected Phrases:")
        for i, phrase in enumerate(self.phrases):
            print(f"Phrase {i+1}: {phrase}")

    def print_logits(self):
        print("\nConfidence:")
        for i, logit in enumerate(self.logits):
            print(f"Logit {i+1}: {logit}")

# --------------------------------------------------------------------------------------------------------------------


# -------------------------------------------- For depth calculation -------------------------------------------------
def map_rgb_to_depth(rgb_x, rgb_y, depth_image):
        """Map an RGB pixel to the corresponding depth pixel and retrieve the depth value."""
        # Step 1: Normalize the RGB pixel coordinates
        normalized_rgb_point = np.linalg.inv(RGB_INTRINSICS).dot([rgb_x, rgb_y, 1])
        
        # Step 2: Transform the normalized point to the depth camera coordinate system
        point_3d_rgb = np.array([normalized_rgb_point[0], normalized_rgb_point[1], 1, 1])
        point_3d_depth = np.linalg.inv(EXTRINSIC_MATRIX).dot(point_3d_rgb)
        point_3d_depth /= point_3d_depth[3]  # Normalize homogeneous coordinates
        
        # Step 3: Project the 3D point to the 2D depth image plane
        x_d = (point_3d_depth[0] * DEPTH_INTRINSICS[0, 0] / point_3d_depth[2]) + DEPTH_INTRINSICS[0, 2]
        y_d = (point_3d_depth[1] * DEPTH_INTRINSICS[1, 1] / point_3d_depth[2]) + DEPTH_INTRINSICS[1, 2]
        
        # Step 4: Undistort the depth point
        undistorted_depth_point = cv2.undistortPoints(np.array([[x_d, y_d]], dtype=np.float32), DEPTH_INTRINSICS, DEPTH_DIST_COEFFS, P=DEPTH_INTRINSICS)
        depth_x, depth_y = undistorted_depth_point[0, 0]
        
        # Step 5: Retrieve the depth value
        if 0 <= int(depth_x) < depth_image.shape[1] and 0 <= int(depth_y) < depth_image.shape[0]:
            depth_value = depth_image[int(depth_y), int(depth_x)]
            return depth_value
        else:
            rospy.logerr("The computed depth pixel is out of bounds.")
            return -1
            
# --------------------------------------------------------------------------------------------------------------------
# print("This file is purely for class and function definitions. Please run img_processor_node.py.")