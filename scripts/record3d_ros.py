#!/usr/bin/env python
import rospy
import sys
import numpy as np
from record3d import Record3DStream
from threading import Event
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Record3DROS:
    def __init__(self):
        rospy.init_node('record3d_ros', anonymous=True)
        self.bridge = CvBridge()
        self.rgb_image_pub = rospy.Publisher("record3d/rgb_image_raw", Image, queue_size=10)
        self.depth_image_pub = rospy.Publisher("record3d/depth_image_raw", Image, queue_size=10)
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__TRUEDEPTH = 0
        self.DEVICE_TYPE__LIDAR = 1
        # ... rest of your initialization code ...

    def on_new_frame(self):
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        rospy.loginfo('Stream stopped')

    def connect_to_device(self, dev_idx):
        rospy.loginfo('Searching for devices')
        print(f"Line 28, Searching for get_connected_devices")
        devs = Record3DStream.get_connected_devices()
        rospy.loginfo('{} device(s) found'.format(len(devs)))
        print('Line 31, {} device(s) found'.format(len(devs)))
        for dev in devs:
            rospy.loginfo('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'
                            .format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)  # Initiate connection and start capturing


    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])

    # ... rest of your methods ...

    def start_processing_stream(self):
        # ... rest of your code ...
        print("Stream processing function has started")
        rospy.loginfo("Stream processing function has started")

        while not rospy.is_shutdown():
            self.event.wait()  # Wait for new frame

            # Copy the newly arrived RGBD frame
            depth = self.session.get_depth_frame()
            rgb = self.session.get_rgb_frame()

            # ... rest of your code ...

            try:
                # Convert your cv::Mat image to a ROS image and publish it
                ros_rgb_img = self.bridge.cv2_to_imgmsg(rgb, "bgr8")
                self.rgb_image_pub.publish(ros_rgb_img)

                # Convert your depth data to a ROS Image message and publish it
                ros_depth_img = self.bridge.cv2_to_imgmsg(depth, "32FC1")
                self.depth_image_pub.publish(ros_depth_img)

            except CvBridgeError as e:
                print(e)

            self.event.clear()

    def run(self):
        self.connect_to_device(dev_idx=0)
        self.start_processing_stream()


if __name__ == '__main__':
    try:
        ros_node = Record3DROS()
        ros_node.run()
    except rospy.ROSInterruptException:
        pass
