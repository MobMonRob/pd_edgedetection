import numpy as np
import time

import cv2
import roslib
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from JsonManager import JsonManager
from ObjectManager import ObjectManager

roslib.load_manifest('edge_detection_pmd')


class Detector:

    def __init__(self):
        rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback)
	rospy.Subscriber("/royale_camera_driver/point_cloud", PointCloud2, self.save_point_cloud)
        self.cv_bridge = CvBridge()

        # print("OpenCV version: {0}".format(cv2.__version__))  # 2.4.8
        self.settings = JsonManager("../parameters/settings.json").load_json()
        self.object_manager = ObjectManager(self.settings["objects"])

        self.timestamp_last_call = time.time()
        self.state = None
        self.pressed_key = -1

        self.colors = {"blue": (255, 0, 0), "green": (0, 255, 0), "red": (0, 0, 255), "yellow": (0, 255, 255),
                       "pink": (255, 0, 255)}

        self.image_bw = None
        self.image_rgb = None
        self.contours = None

        rospy.loginfo("To enable/disable debugging images, press D")
        self.debugging = self.settings["debugging"]
    def save_point_cloud(self, point_cloud):
	self.point_cloud = point_cloud
    def callback(self, img_msg):
        # Prevent running multiple callbacks at once
        if time.time() > self.timestamp_last_call + 1:  # time.time() in seconds
            self.timestamp_last_call = time.time()

            self.convert_img_msg(img_msg)
            self.get_contours()
            self.state()

            # Detect user shutdown
            if self.pressed_key == 27:  # 27 = Escape key
                rospy.signal_shutdown("User Shutdown")
            # Detect debugging
            if self.pressed_key == ord('d'):
                self.debugging = not self.debugging
                rospy.loginfo("Debugging: " + str(self.debugging))

    def get_matching_contours(self, contour_object):
        biggest_difference = self.settings["maximal_contour_difference"]
        matching_contours = {}

        for index, contour_in_scene in enumerate(self.contours):
            difference = cv2.matchShapes(contour_in_scene, contour_object, 1, 0.0)
            if difference < biggest_difference:
                matching_contours[index] = difference

        return matching_contours

    def get_center_on_image(self, contour_index):
        # noinspection PyPep8Naming
        M = cv2.moments(self.contours[contour_index])
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # cv2.circle(self.image_rgb, (cx, cy), 1, self.colors["red"])

        return cx, cy

    def get_object_parameters(self, contour_index):
        center, end_point1, end_point2, rotation = self.get_points_on_vertical(contour_index)

        finger_point1 = self.get_finger_position(center, end_point1, contour_index)
        finger_point2 = self.get_finger_position(center, end_point2, contour_index)

        midpoint_x = int(np.average([finger_point1[0], finger_point2[0]]))
        midpoint_y = int(np.average([finger_point1[1], finger_point2[1]]))
        midpoint = (midpoint_x, midpoint_y)

        cv2.circle(self.image_rgb, midpoint, 1, self.colors["red"])
	vx, vy = self.get_object_axis(contour_index)
	[center, fp1, fp2] = self.get_3d_points([midpoint, finger_point1, finger_point2])

        return midpoint, finger_point1, finger_point2, rotation, vx, vy, center

    def get_finger_position(self, center, direction_point, contour_index):
        intersection = self.intersect_line_with_contour(center, direction_point, contour_index)
        direction_vector_length = self.get_distance(intersection, direction_point)
        finger_x = int(intersection[0] - ((direction_point[0] - intersection[0]) / direction_vector_length) * 1)
        finger_y = int(intersection[1] - ((direction_point[1] - intersection[1]) / direction_vector_length) * 1)
        finger_point = (finger_x, finger_y)
        # cv2.circle(self.image_rgb, intersection, 3, color=self.colors["yellow"], thickness=1)
        cv2.circle(self.image_rgb, finger_point, 4, color=self.colors["red"], thickness=1)
        return finger_point

    def get_points_on_vertical(self, contour_index):
        m = -1 / self.get_rotation(contour_index)
        center = self.get_center_on_image(contour_index)
        cx, cy = center
        rows, cols = self.image_rgb.shape[:2]
        # First point
        x1 = 0
        y1 = int(cy - cx * m)
        # Second point
        x2 = cols - 1
        y2 = int(cy + (x2 - cx) * m)

        angle_rad = np.arctan(m)
        angle_deg = angle_rad * (180 / np.pi)

        return center, (x1, y1), (x2, y2), angle_deg

    def intersect_line_with_contour(self, line_start, line_end, contour_index):
        contour_pixels = np.zeros(self.image_rgb.shape[0:2])
        line_pixels = np.zeros(self.image_rgb.shape[0:2])

        cv2.drawContours(contour_pixels, self.contours, contour_index, 1)
        cv2.line(line_pixels, line_start, line_end, 1, thickness=2)

        intersection = np.logical_and(contour_pixels, line_pixels)

        intersection_x = int(np.average(np.where(intersection)[1]))
        intersection_y = int(np.average(np.where(intersection)[0]))

        # debug = np.zeros(self.image_rgb.shape[0:2]).astype("u1")
        # debug = cv2.cvtColor(debug, cv2.COLOR_GRAY2RGB)
        # cv2.drawContours(debug, self.contours, contour_index, self.colors["yellow"])
        # cv2.line(debug, line_start, line_end, self.colors["blue"], thickness=2)
        # cv2.circle(debug, (intersection_x, intersection_y), 5, color=self.colors["red"], thickness=1)
        # show_image(debug, "Intersection")

        return intersection_x, intersection_y
    #get_object_axis returns two values vx, vy which build together a normalized vector collinear to the fitted line
    def get_object_axis(self, contour_index):
	vx, vy, _, _ = cv2.fitLine(self.contours[contour_index], cv2.DIST_L2, 0, 0.01, 0.01)
	
 	return vy, vx

    def get_rotation(self, contour_index):
	# (vx, vy) is a normalized vector collinear to the line, other return parameters give a point on the line but are not needed
        vx, vy, _, _ = cv2.fitLine(self.contours[contour_index], cv2.DIST_L2, 0, 0.01, 0.01)

        return vy / vx

    def get_contours(self):
        # Delete areas further away than ...
        thresh = (self.settings["camera_thresh"] - self.settings["camera_min"]) * 255 / \
                 (self.settings["camera_max"] - self.settings["camera_min"])

        ret, prepared_image = cv2.threshold(self.image_bw, thresh, 0, cv2.THRESH_TOZERO_INV)
        # Remove noise
        prepared_image = cv2.morphologyEx(prepared_image, cv2.MORPH_OPEN,
                                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
	#prepared_image = cv2.Canny(prepared_image, 100, 200)
        

        self.debug_image(prepared_image, "Prepared Image")

        # Find Contours
        image, contours, hierarchy = cv2.findContours(prepared_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Filter useful contours
        self.contours = []
        image_y, image_x = self.image_bw.shape
        corners = [(2, 2), (2, image_y - 3), (image_x - 3, 2), (image_x - 3, image_y - 3)]
        for contour in contours:
            # Only select contours with more than 40 points
            if len(contour) > self.settings["minimal_contour_length"]:
                in_corner = False
                # Test if Contour is in one of the corners
                for point in corners:
                    if cv2.pointPolygonTest(contour, point, False) == 1:
                        in_corner = True
                if not in_corner:
                    self.contours.append(contour)

        # Sort useful contours by their number of points
        self.contours.sort(key=len, reverse=True)

        # Show useful contours
        image_show = np.copy(self.image_rgb)
        #image_show = np.copy(prepared_image)
        # drawContours(image, contours, contourIdx, color, thickness)
        cv2.drawContours(image_show, self.contours, -1, (0, 255, 255), 1)
        for corner in corners:
            cv2.circle(image_show, corner, 1, color=self.colors["blue"], thickness=1)
        self.debug_image(image_show, "Useful Contours")

    def convert_img_msg(self, img_msg):
        # Convert ros image message to numpy array
        float_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")

        # Adjust Image
        float_image[float_image < 0.01] = self.settings["camera_thresh"]
        float_image = (float_image - self.settings["camera_min"]) / \
                      (self.settings["camera_max"] - self.settings["camera_min"])
        float_image[float_image > 1] = 1
        float_image[float_image < 0] = 0

        # Convert 32fc1 to 8uc1
        self.image_bw = (float_image * 255).astype('u1')
        self.image_rgb = cv2.cvtColor(self.image_bw, cv2.COLOR_GRAY2RGB)

    def show_image_wait(self, window_name):
        show_image(self.image_rgb, window_name)
        self.pressed_key = cv2.waitKey(500) & 255

    def debug_image(self, image, window_name):
        if self.debugging:
            show_image(image, window_name)

    def draw_contour(self, contour_index, color_index=2):
        cv2.drawContours(self.image_rgb, self.contours, contour_index, self.colors.values()[color_index], 1)

    def get_3d_points(self, points_2d):
        points_3d = []
        for point_3d in list(pc2.read_points(self.point_cloud, uvs=points_2d)):
            x, y, z = point_3d
            points_3d.append(Point(x, y, z))
	return points_3d

    @staticmethod
    def get_distance(point_1, point2):
        return np.sqrt(np.power(point_1[0] - point2[0], 2) + np.power(point_1[1] - point2[1], 2))


def show_image(image, window_name):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, image.shape[1] * 4, image.shape[0] * 4)
    cv2.imshow(window_name, image)  # Show image
