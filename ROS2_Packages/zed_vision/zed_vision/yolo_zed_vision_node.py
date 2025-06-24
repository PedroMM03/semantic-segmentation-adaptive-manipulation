'''
Project name: Semantic Segmentation for Adaptive Manipulation with Industrial Robots
Author: Pedro Martin
Electronics, Robotics and Mechatronics Engineering - University of Malaga
Date: 2025
 
'''

# Import standard Python libraries
from cmath import nan
from email.mime import image
from multiprocessing.pool import IMapIterator
from pickle import TRUE

# ROS2 core libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS2 utilities
from ament_index_python.packages import get_package_share_directory

# ROS2 message types for geometry
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped

# ROS2 standard message types
import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int16

# ROS2 sensor message types
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image

# Utility to convert between ROS images and OpenCV format
from cv_bridge import CvBridge

# Enum utilities
from enum import Flag

# OpenCV for image processing
import cv2

# JSON for configuration or data serialization
import json

# ZED SDK for camera interaction
import pyzed.sl as sl

# Numerical processing
import numpy as np
import struct
import ctypes
import os

# YOLO model and utilities from Ultralytics
from ultralytics import solutions, YOLO

# Image processing filters
from scipy import ndimage

'''
# Optional backend setting for matplotlib
import matplotlib
matplotlib.use('TkAgg')
'''

# Matplotlib for data visualization (3D plotting)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D



class ComputerVision(Node):

    def __init__(self):
        super().__init__('yolo_zed_vision_node')  # Initialize the ROS2 node with name 'ComputerVision'
        timer_period = 0.2  # Node will execute the timer_callback every 0.2 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Global parameters
        self.declare_parameter('conf', 0.5)          
        self.declare_parameter('check_radio', 30)

        # Global class variables used for object selection and verification logic
        self.operation_mode=0
        self.selected_obj_id = -1
        self.selected_obj_class = ''
        self.ref_check_point = (0,0)
        self.check_radio = self.get_parameter('check_radio').value   # Radius in pixels for object checking
        self.checked_object = False
        self.checked_object_counter = 0
        self.conf = self.get_parameter('conf').value
        # ROS2 CONFIGURATION

        # Publisher definitions with custom QoS settings for reliable and transient local delivery
        qos_profile= QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher for detected object position (3D coordinates)
        self.obj_position_pub=self.create_publisher(PointStamped,'/zed_vision/object_position',qos_profile)
        # Publisher for detected object orientation (as a 3D vector)
        self.obj_orientation_pub=self.create_publisher(Vector3Stamped,'/zed_vision/object_orientation',qos_profile)
        # Publisher for segmented object data in JSON format
        self.segmented_objs_json_pub = self.create_publisher(String, '/zed_vision/segmented_objects',qos_profile)
        # Publisher to trigger object verification
        self.check_object_pub=self.create_publisher(Bool,'/zed_vision/check_object',qos_profile)

        # OpenCV bridge to convert between ROS Image messages and OpenCV images
        self.bridge=CvBridge()
        # Publisher for annotated camera frames (with detections)
        self.annotated_frame_pub = self.create_publisher(Image,'/zed_vision/annotated_frame',10)

        # Data structure to store information about segmented objects
        self.segmented_objs_json=[]

        # Subscriptions to external ROS2 topics (e.g., control commands, UI selections)
        self.process_step_sub=self.create_subscription(Int16,'/xarm6_controller/process_step',self.process_step_cb,qos_profile)
        self.selected_object_sub = self.create_subscription(String,'/segmentation_panel/selected_object',self.selected_object_sub_cb,qos_profile)

        # 1. ZED camera configuration and initialization
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  # Set resolution to 720p
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA   	# Enable high-quality depth mode
        self.init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeters for distance units

        self.MAX_DEPTH=5000   # Maximum depth threshold in mm

        # Attempt to open the ZED camera with the given parameters
        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Error opening ZED camera")
            exit()

        # Retrieve the intrinsic parameters of the left camera
        self.calib_params=self.zed.get_camera_information()
        self.intrinsics=self.calib_params.camera_configuration.calibration_parameters.left_cam

        # Store the intrinsic camera parameters for later use in projection/math
        self.fx = self.intrinsics.fx
        self.fy = self.intrinsics.fy
        self.cx = self.intrinsics.cx
        self.cy = self.intrinsics.cy
        self.width = self.intrinsics.image_size.width
        self.height = self.intrinsics.image_size.height

        print(f"Intrinsics:\n fx: {self.fx}, fy: {self.fy}, cx: {self.cx}, cy: {self.cy}")
        print(f"image size: {self.width}x{self.height}")

        # 2. Load the YOLO model for object detection
        pkg_path=get_package_share_directory('zed_vision')
        model_path = os.path.join(pkg_path,'resource','models','yolo11l-seg.pt')
        engine_path = os.path.join(pkg_path,'resource','models','yolo11l-seg.engine')

        # If the TensorRT engine doesn't exist yet, export the YOLO model to engine format
        if not os.path.exists(engine_path):
            self.model = YOLO(model_path)
            self.model.export(format="engine", imgsz=(736, 1280), half=True, device="dla:0",)

        # Load the pre-exported YOLO engine for inference
        self.model = YOLO(engine_path)

        # Path to configuration file for the BoT-SORT tracking algorithm
        self.tracker_path = os.path.join(pkg_path,'config','botsort.yaml')

        # 3. Initialize ZED matrices to store depth map and point cloud data
        self.depth_map = sl.Mat()   
        self.point_cloud = sl.Mat()	 
        self.primera_vez=True  # Used to control first-time initialization logic

        # Create OpenCV windows for visualization (debugging and inspection)
        cv2.namedWindow("Segmentation and grasp point", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Segmentated Depth_map", cv2.WINDOW_NORMAL)



    def selected_object_sub_cb(self,msg):
        """
        Callback function triggered when a new message is received on the 
        '/segmentation_panel/selected_object' topic.

        Expects a JSON-formatted string containing the selected object's ID and class.
        Updates internal state with the selected object's data.
        """
        try:
            data = json.loads(msg.data)
            self.selected_obj_id = data.get('id')
            self.selected_obj_class= data.get('class')

            self.get_logger().info(f"Received object -> ID: {self.selected_obj_id}, CLASS: {self.selected_obj_class}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")



    def process_step_cb(self,msg):
        """
        Callback function for the '/xarm6_controller/process_step' topic.

        This message controls the current step of the robot or system process.
        Based on the step value, the node switches between different operation modes:
        - Step 3: Activate orientation estimation mode
        - Step 9: Activate object check mode
        - Other steps: Deactivate both modes (default state)
        """
        self.process_step = msg.data
        # Normal must be calculated from steps 3 to XX
        if  self.process_step == 3:
            print("Orientation activated")
            self.operation_mode=1
        elif self.process_step == 9:
            print("Check mode")
            self.operation_mode=2
        else:
            print("Orientation deactivated")
            self.operation_mode=0


        

    def projection_3d(self, j, i, z, intrinsics):
        """
        Projects a 2D pixel coordinate (j, i) with depth z into 3D space
        using the provided camera intrinsic parameters.

        Parameters:
        - j (int): Horizontal pixel coordinate (u-axis)
        - i (int): Vertical pixel coordinate (v-axis)
        - z (float): Depth value at pixel (in mm)
        - intrinsics: Camera intrinsics containing fx, fy, cx, cy

        Returns:
        - np.array: 3D point [x, y, z] in the camera coordinate frame
        """

        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.cx
        cy = intrinsics.cy
        
        x = (j - cx) * z / fx
        y = (i - cy) * z / fy
        return np.array([x, y, z])





    def colorize_label_map(self, label_map, index_min, index_max):
        """
        Converts a label map into a color image using fixed colors for specific labels.

        Each unique label is assigned a predefined color:
        - index_min: red (closest object)
        - index_max: green (farthest object)
        - all other labels: blue (intermediate objects)
        - label 255: white (background or ignored areas)

        Parameters:
        - label_map (np.ndarray): 2D array containing label values for each pixel
        - index_min (int): Label value corresponding to the closest object
        - index_max (int): Label value corresponding to the farthest object

        Returns:
        - color_img (np.ndarray): 3-channel RGB image with color-coded labels
        """

        # Fixed color assignments (B, G, R format for OpenCV)
        color_dict = {
            0: (0, 0, 255),    # Red
            1: (0, 255, 0),    # Green
            2: (255, 0, 0),    # Blue
            -1: (255, 255, 255)  # Background → White
        }

        h, w = label_map.shape
        color_img = np.zeros((h, w, 3), dtype=np.uint8)

        # Apply color to each unique label in the label map
        for label in np.unique(label_map):
            mask = (label_map == label)
            if label == 255:
                color_img[mask] = color_dict[-1]  # Background
            elif label == index_min:
                color_img[mask] = color_dict[0]   # Closest object → Red
            elif label == index_max:
                color_img[mask] = color_dict[1]   # Farthest object → Green
            else:
                color_img[mask] = color_dict[2]   # Intermediate objects → Blue

        return color_img


    def plot_plane_with_normal(self, points, centroid_3d, normal):  # Only for debugging
        """
        Visualizes a 3D plane fitted to a set of points, along with its normal vector.

        This function is primarily for debugging purposes. It creates a 3D plot that includes:
        - The original 3D points (typically a neighborhood)
        - The plane passing through the centroid with the given normal
        - A red vector showing the direction of the normal

        Parameters:
        - points (np.ndarray): Nx3 array of 3D points
        - centroid_3d (np.ndarray): 3D coordinates of the centroid of the point set
        - normal (np.ndarray): Normal vector of the plane
        """

        # Calculate plane equation: ax + by + cz + d = 0
        d = -centroid_3d.dot(normal)

        # Create a grid of x and y values centered around the centroid
        xx, yy = np.meshgrid(
            np.linspace(centroid_3d[0] - 20, centroid_3d[0] + 20, 10),
            np.linspace(centroid_3d[1] - 20, centroid_3d[1] + 20, 10)
        )

        # Calculate corresponding z values using the plane equation
        zz = (-normal[0]*xx - normal[1]*yy - d) / normal[2]

        # Create 3D figure
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot original 3D points
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='blue')

        # Plot the fitted plane
        ax.plot_surface(xx, yy, zz, color='orange', alpha=0.5)

        # Draw the normal vector starting at the centroid
        ax.quiver(
            centroid_3d[0], centroid_3d[1], centroid_3d[2],
            normal[0], normal[1], normal[2],
            length=20, color='red'
        )

        # Custom legend (manually created to match colors)
        custom_lines = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=8, label='Neighborhood'),
            Line2D([0], [0], color='orange', lw=8, label='Plane'),
            Line2D([0], [0], color='red', lw=2, label='Normal'),
        ]
        ax.legend(handles=custom_lines)

        # Axis labels and title
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Tangent plane and normal at centroid")

        # Show plot with layout adjustment
        plt.tight_layout()
        plt.show()

        # Keep plot open and allow updates (useful during debugging)
        plt.draw()
        plt.pause(0.1)
        plt.show(block=False)




    def find_grasp_point_and_normal_vector(self, depth_map, mask, centroid, window_size=9):
        """
        Uses the centroid as the grasp point and calculates the average normal
        over a square region of side 'window_size' around the centroid.

        Parameters:
            depth_map: Depth map (2D array, e.g. in millimeters).
            mask: Binary mask (same size as depth_map) indicating the object.
            centroid: (X, Y) pixel coordinates from the mask.
            window_size: Size of the square region around the centroid for averaging.
        
        Returns:
            normal: Average normal vector (3 elements, normalized) in the region.
            grasp_point: Grasp point coordinates calculated by projecting the original centroid onto the plane.
        """

        # Flag indicating if a new centroid has been recalculated
        nuevo_centroid = False

        # Grasp point is initially the centroid (rounded to integer)
        # Optional smoothing

        # Mask depth values to keep only object pixels
        depth_masked = np.where(mask > 0, depth_map, 0).astype(np.float32)

        # Extract valid depth pixels (greater than zero)
        valid_pixels = depth_masked[depth_masked > 0].reshape(-1, 1)

        # Check if there are enough valid pixels for KMeans clustering
        if valid_pixels.shape[0] < 3:
            print("Not enough valid pixels to apply KMeans.")
            default_normal = np.array([0.0, 0.0, 1.0])
            default_z = depth_map[int(centroid[0]), int(centroid[1])]
            default_grasp_point = np.array([centroid[1], centroid[0], default_z])
            return default_normal, default_grasp_point, default_grasp_point

        # Apply KMeans clustering with 3 clusters on the valid depth pixels
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
        _, labels, centers = cv2.kmeans(valid_pixels.astype(np.float32), 3, None, criteria, 10, cv2.KMEANS_PP_CENTERS)

        # Assign cluster labels back to a full-size image label map, initialized with 255 (background)
        label_map = np.full_like(depth_masked, fill_value=255, dtype=np.uint8)
        label_map[depth_masked > 0] = labels.flatten()

        # Identify cluster indices for the nearest (min) and farthest (max) clusters based on cluster centers
        min_cluster_index = np.argmin(centers)
        max_cluster_index = np.argmax(centers)

        # Visualize clustering results (for debugging)
        color_img = self.colorize_label_map(label_map, min_cluster_index, max_cluster_index)
        cv2.imshow("Segmentated Depth_map", color_img)
        cv2.waitKey(1)

        print("Clustering centers: ", centers)
        print("Mask centroid: ", centroid)
        
        # Round centroid coordinates to nearest integer
        i, j = np.round(centroid).astype(int)
        h, w = depth_masked.shape

        # Check if the centroid pixel belongs to the nearest cluster
        if label_map[i, j] != min_cluster_index:
            print("Centroid not in nearest cluster. Searching for closest point...")

            cluster_min = np.argwhere(label_map == min_cluster_index)
            if cluster_min.ndim == 1:
                cluster_min = cluster_min.reshape(1, -1)

            centroid_px = np.array([i, j])
            dist = np.linalg.norm(cluster_min - centroid_px, axis=1)

            centroid_nearest_idx = np.argmin(dist)
            centroid_nearest = cluster_min[centroid_nearest_idx]

            # Update centroid to nearest cluster pixel for plane and normal calculation
            i, j = int(centroid_nearest[0]), int(centroid_nearest[1])
            print("New centroid point: ", (i, j))
            nuevo_centroid = True

        # Define the neighborhood window around the centroid
        half = window_size // 2
        i_min = max(i - half, 0)
        i_max = min(i + half + 1, h)
        j_min = max(j - half, 0)
        j_max = min(j + half + 1, w)

        # Collect valid points within the neighborhood that belong to the nearest cluster
        points = []
        for ii in range(i_min, i_max):
            for jj in range(j_min, j_max):
                if label_map[ii, jj] == min_cluster_index:
                    z = depth_masked[ii, jj]
                    if z > 0 and np.isfinite(z):
                        points.append([jj, ii, z])  # (x, y, z) with x=column, y=row

        # Check if there are enough points to fit a plane
        if len(points) < 3:
            print("Not enough points to estimate a plane")
            default_normal = np.array([0.0, 0.0, 1.0])
            default_z = depth_map[i, j]
            default_centroid = np.array([j, i, default_z])
            return default_normal, default_centroid

        # Fit a plane to the points using Singular Value Decomposition (SVD)
        pts = np.array(points)
        centroid_3d = np.mean(pts, axis=0)  # Mean point (x, y, z)
        pts_centered = pts - centroid_3d

        # SVD to find normal vector (direction of least variance)
        _, _, vh = np.linalg.svd(pts_centered)
        normal = vh[-1]  # Normal vector is last row of vh

        # Ensure the normal is pointing towards the camera (optional)
        if normal[2] > 0:
            normal = -normal

        # Normalize the normal vector
        norm_val = np.linalg.norm(normal)
        if norm_val > 1e-6:
            normal /= norm_val
        else:
            normal = np.array([0.0, 0.0, 1.0])

        # Optional: visualize the plane and normal with the selected points
        # plot_plane_with_normal(pts, centroid_3d, normal)

        # If a new centroid was chosen, compute two grasp points:
        # project the original centroid onto the estimated plane
        if nuevo_centroid:
            centroid_global = np.round(centroid).astype(float)
            y_global, x_global = centroid_global  # centroid as (row, col)
            x0, y0, z0 = centroid_3d
            nx, ny, nz = normal

            # Avoid division by zero when calculating z on plane
            if abs(nz) < 1e-6:
                z_plane = z0
            else:
                z_plane = z0 - (nx * (x_global - x0) + ny * (y_global - y0)) / nz

            grasp_point= np.array([x_global, y_global, z_plane])
            print("Grasp point method 1: ", grasp_point)
            
        else:
            z_centroid = depth_map[i, j]
            centroid_3d = np.array([j, i, z_centroid])
            grasp_point = centroid_3d
            grasp_point = centroid_3d

        return normal, grasp_point



    def opencv_visualization(self, frame, grasp_point, normal, scale=150, visible_normal=False):
        """
        Visualizes the grasp point(s) and optionally the normal vector on the given image frame.
        
        Parameters:
            frame: Image (BGR) where to draw.
            grasp_point: 3D grasp point [x, y, z]; x and y used for visualization.
            normal: Normal vector (3 elements).
            scale: Scale factor to visualize the normal vector.
            visible_normal: Boolean to decide whether to draw the normal vector.
        """
        # Convert grasp points coordinates to integers for drawing
        grasp_x, grasp_y, grasp_z = [int(round(coord)) for coord in grasp_point]

        normal_x, normal_y, normal_z = normal  # These remain floats

        print("Grasp points visualization coordinates:", grasp_x, grasp_y)

        # Draw grasp point (green circle)
        cv2.circle(frame, (grasp_x, grasp_y), 10, (0, 255, 0), -1)

        if visible_normal:
            # Calculate the endpoint of the normal vector arrow in 2D (only x and y components)
            end_x = int(round(grasp_x + normal_x * scale))
            end_y = int(round(grasp_y + normal_y * scale))

            # Draw an arrow representing the normal vector (red color)
            cv2.arrowedLine(frame, (grasp_x, grasp_y), (end_x, end_y), (0, 0, 255), 2)

        # Convert annotated OpenCV image to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera"
        self.annotated_frame_pub.publish(image_msg)


    def pointcloud_to_ros(self, zed_point_cloud):
        """
        Converts a ZED point cloud (H, W, 4) to a ROS2 PointCloud2 message.

        Parameters:
            zed_point_cloud: object with get_data() method returning an (H, W, 4) array with [X,Y,Z,intensity].

        Returns:
            pc2: sensor_msgs/PointCloud2 message ready to be published in ROS2.

        Note:
            This function currently only converts and returns the message,
            it is NOT currently used to publish point clouds in ROS2 in the current node.
        """

        points = zed_point_cloud.get_data()  # (H, W, 4) in millimeters
        h, w, _ = points.shape

        # Convert to meters and flatten to (N, 3)
        pc_flat = points[:, :, :3].reshape(-1, 3) / 1000.0

        # Remove points with NaNs
        pc_valid = pc_flat[~np.isnan(pc_flat).any(axis=1)]

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera"  # Fixed frame used by the camera

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = pc_valid.astype(np.float32).tobytes()

        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(pc_valid)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12  # 3 floats * 4 bytes each
        pc2.row_step = pc2.point_step * len(pc_valid)
        pc2.data = cloud_data
        pc2.is_dense = True

        return pc2
    


    # 4. Main loop callback executed periodically by a timer
    def timer_callback(self):

        # Attempt to grab a new frame from the ZED camera
        if self.zed.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image from the stereo camera
            left_image = sl.Mat()
            self.zed.retrieve_image(left_image, sl.VIEW.LEFT)
            frame = left_image.get_data()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Convert image from BGRA to BGR color space

            # Convert BGR image to HSV for brightness adjustment
            frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            h,s,v = cv2.split(frame_hsv)

            # Reduce brightness by subtracting from the value channel
            v = cv2.subtract(v, 100)
            v = np.clip(v, 0, 255)

            # Merge channels back and convert to BGR again
            frame_hsv = cv2.merge([h,s,v])
            frame_bgr = cv2.cvtColor(frame_hsv, cv2.COLOR_HSV2BGR)

            # Perform object detection and tracking with YOLO model
            results = self.model.track(
                source=frame_bgr,
                imgsz=(736, 1280),   # YOLO model requires image sizes multiple of 32
                conf=self.conf,            # Confidence threshold
                classes=[39,46,47,67],  # Filter only specific classes of interest
                device="cuda:0",     # Use GPU for inference
                show=False,
                retina_masks=True,   # Use high-quality segmentation masks
                verbose=False,
                tracker=self.tracker_path,
                persist=True
            )

            # Automatically annotate detected objects with masks and bounding boxes
            frame_anotado = results[0].plot(
                conf=True,
                labels=True,
                masks=True,
                boxes=True,
                color_mode='instance'
            )

            # Retrieve depth map and 3D point cloud from the camera
            self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)

            # Convert depth map to numpy array, replace NaNs and infinities
            depth_map_np = np.nan_to_num(
                self.depth_map.get_data(),
                nan=0.0,
                posinf=self.MAX_DEPTH,
                neginf=0.0
            )

            # Process detection results if available
            if results and len(results) > 0:

                result = results[0]

                # Check if valid boxes and masks are present
                if result.boxes is not None and result.masks is not None and hasattr(result.boxes, 'id') and result.boxes.id is not None:

                    # Extract object IDs, class IDs, and masks from the result
                    obj_ids = results[0].boxes.id.cpu().numpy().astype(int)
                    classes = results[0].boxes.cls.cpu().numpy().astype(int)
                    classes_names = [self.model.names[c] for c in classes]
                    masks = results[0].masks.data.cpu().numpy().astype(np.uint8)

                    if masks.size > 0:
                        # Iterate over each detected object mask
                        for i in range(len(masks)):

                            mask_selected = masks[i]
                            # Resize mask to match annotated frame size
                            mask_selected = cv2.resize(mask_selected, (frame_anotado.shape[1], frame_anotado.shape[0]))
                            class_name = classes_names[i]
                            obj_id = int(obj_ids[i])

                            # Verify that mask size matches depth map size
                            if mask_selected.shape == depth_map_np.shape:

                                # Compute centroid of the mask pixels
                                mask_centroid = np.mean(np.argwhere(mask_selected), axis=0)

                                # Handle different operation modes for processing
                                if self.operation_mode == 1:

                                    # In mode 1, only calculate grasp point for selected object ID
                                    if self.selected_obj_id == obj_id:
                                        normal, grasp_point = self.find_grasp_point_and_normal_vector(depth_map_np, mask_selected, mask_centroid)
                                        self.ref_check_point = mask_centroid

                                        print("Calculated normal vector: ", normal)

                                        # Visualize grasp point and normal on the frame
                                        self.opencv_visualization(frame_anotado, grasp_point, normal, 150, True)

                                        # Project 2D grasp point into 3D space using camera intrinsics
                                        grasp_point_3d = self.projection_3d(grasp_point[0], grasp_point[1], grasp_point[2], self.intrinsics)
                                        print("Grasp point 3D coordinates: ", grasp_point_3d)

                                        # Publish the grasp point position relative to camera frame (meters)
                                        msg_position = PointStamped()
                                        msg_position.header.stamp = self.get_clock().now().to_msg()
                                        msg_position.header.frame_id = 'camera'
                                        msg_position.point.x = grasp_point_3d[0] / 1000
                                        msg_position.point.y = grasp_point_3d[1] / 1000
                                        msg_position.point.z = grasp_point_3d[2] / 1000

                                        self.obj_position_pub.publish(msg_position)
                                        print("Grasp point published: ", msg_position.point.x, msg_position.point.y, msg_position.point.z)

                                        # Publish the normal vector orientation
                                        msg_orientation = Vector3Stamped()
                                        msg_orientation.header.stamp = self.get_clock().now().to_msg()
                                        msg_orientation.header.frame_id = 'camera'
                                        msg_orientation.vector.x = normal[0]
                                        msg_orientation.vector.y = normal[1]
                                        msg_orientation.vector.z = normal[2]

                                        self.obj_orientation_pub.publish(msg_orientation)
                                        print("Normal vector published: ", msg_orientation.vector.x, msg_orientation.vector.y, msg_orientation.vector.z)

                                    else:
                                        print("Selected ID:", self.selected_obj_id, "Current object ID:", obj_id)

                                elif self.operation_mode == 0:
                                    # Mode 0 uses centroid coordinates with depth to approximate position
                                    mask_centroid = (int(mask_centroid[0]), int(mask_centroid[1]))
                                    z_centroid = depth_map_np[mask_centroid[0], mask_centroid[1]]

                                    # Use centroid depth if valid, else fallback to SVD estimation
                                    if not np.isnan(z_centroid) and z_centroid > 0:
                                        mask_centroid_3d_image = np.array([mask_centroid[1], mask_centroid[0], z_centroid])
                                    else:
                                        _, mask_centroid_3d_image = self.find_grasp_point_and_normal_vector(depth_map_np, mask_selected, mask_centroid)

                                    # Visualize centroid point without normal vector
                                    print("Centroid 3D point: ", mask_centroid_3d_image)
                                    normal = np.array([0, 0, 0])  # Default normal vector for visualization
                                    self.opencv_visualization(frame_anotado, mask_centroid_3d_image, mask_centroid_3d_image, normal, False)

                                    # Project centroid point into 3D space
                                    aproximation_point_3d = self.projection_3d(mask_centroid_3d_image[0], mask_centroid_3d_image[1], mask_centroid_3d_image[2], self.intrinsics)

                                    # Calculate Euclidean distance from the camera
                                    x = float(aproximation_point_3d[0]) / 1000
                                    y = float(aproximation_point_3d[1]) / 1000
                                    z = float(aproximation_point_3d[2]) / 1000
                                    dist = np.linalg.norm([x, y, z])

                                    # Register detected object info for JSON message
                                    register = {
                                        "id": obj_id,
                                        "class": class_name,
                                        "x": x,
                                        "y": y,
                                        "z": z,
                                        "dist": dist
                                    }

                                    # Append detected object info to list for publishing
                                    self.segmented_objs_json.append(register)

                                elif self.operation_mode == 2:
                                    # Mode 2 checks if an object of a selected class is within a certain radius

                                    if class_name == self.selected_obj_class:
                                        # Compute distance between mask centroid and reference point
                                        dist_from_centroid_to_refpoint = np.linalg.norm(np.array(mask_centroid) - np.array(self.ref_check_point))
                                        if dist_from_centroid_to_refpoint <= self.check_radio:
                                            self.checked_object = True
                                            print("Object detected")

                            else:
                                print("Error: Mask and depth map sizes do not match.")
                                print("Mask size:", mask_selected.shape)
                                print("Depth map size:", depth_map_np.shape)

                    # After processing all masks, handle mode 2's boolean publishing and visualization
                    if self.operation_mode == 2:
                        msg_bool = Bool()
                        msg_bool.data = self.checked_object
                        self.check_object_pub.publish(msg_bool)
                        print("Checked object status: ", self.checked_object)

                        # Draw region of interest circle around reference check point
                        center = (int(self.ref_check_point[1]), int(self.ref_check_point[0]))
                        print("Center coordinates: ", center)
                        radius = int(self.check_radio)
                        color = (0, 255, 0) if self.checked_object else (0, 0, 255)
                        thickness = 2
                        cv2.circle(frame_bgr, center, radius, color, thickness)

                        # Draw detection status text
                        text = "OBJECT DETECTED" if self.checked_object else "NO OBJECT DETECTED"
                        text_posititon = (center[0] - radius, center[1] - radius - 10)
                        cv2.putText(frame_bgr, text, text_posititon, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

                        # Publish annotated frame as ROS Image message
                        image_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                        image_msg.header.stamp = self.get_clock().now().to_msg()
                        image_msg.header.frame_id = "camera"
                        self.annotated_frame_pub.publish(image_msg)

                        # Reset checked object flag for next iteration
                        self.checked_object = False

                    # Publish detected objects information as JSON string message
                    # Sort objects by distance before publishing
                    self.segmented_objs_json.sort(key=lambda obj: obj["dist"])
                    msg_json = String()
                    msg_json.data = json.dumps(self.segmented_objs_json)
                    self.segmented_objs_json_pub.publish(msg_json)
                    self.segmented_objs_json.clear()

                else:
                    # Handle case when no valid detection data is available
                    print("No valid data")

                    # Reset checked object flag and publish false status
                    self.checked_object = False
                    msg_bool = Bool()
                    msg_bool.data = self.checked_object
                    self.check_object_pub.publish(msg_bool)

                    if (self.operation_mode == 2):
                        # Draw red circle indicating no detection in mode 2
                        center = (int(self.ref_check_point[1]), int(self.ref_check_point[0]))
                        radius = int(self.check_radio)
                        color = (0, 0, 255)
                        thickness = 2
                        cv2.circle(frame_bgr, center, radius, color, thickness)

                        # Draw "NO OBJECT DETECTED" text
                        text = "NO OBJECT DETECTED"
                        text_posititon = (center[0] - radius, center[1] - radius - 10)
                        cv2.putText(frame_bgr, text, text_posititon, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

                    # Publish the image frame even when no detection is found
                    image_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = "camera"
                    self.annotated_frame_pub.publish(image_msg)

                    # Clear and publish empty JSON message since no objects were detected
                    msg_json = String()
                    self.segmented_objs_json.clear()
                    msg_json.data = json.dumps(self.segmented_objs_json)
                    self.segmented_objs_json_pub.publish(msg_json)

            else:
                # In case the grab fails or no result, publish the annotated frame as is
                image_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = "camera"
                self.annotated_frame_pub.publish(image_msg)

            # Display the annotated frame in an OpenCV window
            cv2.imshow("Segmentation and grasp point", frame_anotado)
            cv2.waitKey(1)





    

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the ComputerVision node
    node = ComputerVision()

    try:
        # Keep the node running, processing callbacks until shutdown or interrupt
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle user interrupt (Ctrl+C) gracefully
        node.get_logger().info('Stop execution by user (Ctrl+C)')
    finally:
        # Cleanup before exiting:
        # Close the ZED camera connection
        node.zed.close()
        # Close all OpenCV windows
        cv2.destroyAllWindows()
        # Destroy the ROS node explicitly
        node.destroy_node()
        # Shutdown ROS client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
