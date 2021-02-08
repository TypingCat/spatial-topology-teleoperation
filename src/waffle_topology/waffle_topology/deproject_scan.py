#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32

import cv2
import numpy as np
import math

class Deproject_scan(Node):
    def __init__(self):
        # Set parameters
        self.scan_angle_min = -3.14
        self.scan_angle_max = 3.14
        self.scan_radius = 5.0
        self.scan_map_resolution = 0.05
        self.occupancy_img_threshold = 127

        self.scan = LaserScan()
        self.scan_map_shape = (
            int((2*self.scan_radius) // self.scan_map_resolution),
            int((2*self.scan_radius) // self.scan_map_resolution), 1)
        self.scan_map_center = (
            int(self.scan_map_shape[0] // 2),
            int(self.scan_map_shape[1] // 2))

        # Initialize ROS node
        super().__init__('waffle_topology_deproject_scan')
        self.scan_sample_publisher = self.create_publisher(PointCloud, '/scan/sample', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.area_publisher = self.create_publisher(OccupancyGrid, '/topology/area', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
    
    def scan_callback(self, msg):
        self.scan = msg

    def timer_callback(self):
        """Convert scan to map"""

        # Deproject scan ranges to points
        angle = self.scan.angle_min - self.scan.angle_increment + math.pi
        points = []
        for r in self.scan.ranges:
            angle += self.scan.angle_increment
            angle = (angle + math.pi) % (2*math.pi) - math.pi    
            if angle < self.scan_angle_min or angle > self.scan_angle_max:
                r = self.scan.range_min
            elif r >= self.scan.range_max or r <= self.scan.range_min:
                continue
            points.append([
                self.scan_map_center[1] - r * math.sin(angle) / self.scan_map_resolution,
                self.scan_map_center[0] - r * math.cos(angle) / self.scan_map_resolution])

        # Sampling points
        points_size = 100
        points_step = len(points) // points_size
        points_step = max(1, points_step)
        points = points[0::points_step]

        # Publish area
        image_scan = np.zeros(self.scan_map_shape, np.uint8)
        try:
            cv2.fillPoly(image_scan, [np.asarray(points, dtype='int32')], 255)
        except:
            return
        image_scan = self.morphology_filter(image_scan, 3)
        area = self.convert_image_to_map(image_scan, self.scan)
        self.area_publisher.publish(area)
        print("Publish scan area", area.header.stamp.sec)

        # Publish samples
        scan_sample = PointCloud()
        scan_sample.header = self.scan.header
        for point in points:
            p = Point32(
                x = (point[1] - self.scan_map_center[1]) * self.scan_map_resolution,
                y = (point[0] - self.scan_map_center[0]) * self.scan_map_resolution,
                z = 0.3)
            scan_sample.points.append(p)
        self.scan_sample_publisher.publish(scan_sample)

    def morphology_filter(self, img, num):
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        for i in range(num):
            img = cv2.erode(img, element)
        for i in range(num):
            img = cv2.dilate(img, element)
            img = cv2.dilate(img, element)
        for i in range(num):
            img = cv2.erode(img, element)
        return img

    def convert_image_to_map(self, img, msg):
        m = OccupancyGrid()
        m.header = msg.header
        m.header.frame_id = "base_scan"
        m.info.map_load_time = self.get_clock().now().to_msg()
        m.info.resolution = 0.05
        m.info.width = self.scan_map_shape[0]
        m.info.height = self.scan_map_shape[1]
        m.info.origin.position.x = -self.scan_radius
        m.info.origin.position.y = -self.scan_radius
        m.info.origin.orientation.w = 1.0
        m.data = [-1 for i in range(self.scan_map_shape[0] * self.scan_map_shape[1])]
        for i in range(m.info.width):
            for j in range(m.info.height):
                n = m.info.width * j + i
                if img[i, j] > self.occupancy_img_threshold:
                    m.data[n] = 0   # {-1: unknown, 0: free, 100: occupied}
        return m

def main(args=None):
    rclpy.init(args=args)
    node = Deproject_scan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
