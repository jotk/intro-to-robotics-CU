#!/usr/bin/env python


import argparse
import numpy as np
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface

class ArmCamera:
    def __init__(self):
        self.cv_image = None
        self.lego_coords = []
        self.main()


    def show_image_callback(self, img_data, (edge_detection, window_name)):
        """The callback function to show image by using CvBridge and cv
        """
        print("iterating...")
        bridge = CvBridge()
        try:
            # print("TYPE", type(img_data))
            self.cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
        except CvBridgeError, err:
            rospy.logerr(err)
            return
        if edge_detection == True:
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            # customize the second and the third argument, minVal and maxVal
            # in function cv2.Canny if needed
            get_edge = cv2.Canny(blurred, 10, 100)
            self.cv_image = np.hstack([get_edge])
        edge_str = "(Edge Detection)" if edge_detection else ''
        cv_win_name = ' '.join([window_name, edge_str])
        cv2.namedWindow(cv_win_name, 0)
        # refresh the image on the screen
        cv2.imshow(cv_win_name, self.cv_image)
        self.set_lego_locations()
        cv2.waitKey(99999999)

    def set_lego_locations(self):
        if self.cv_image == None:
            return -1
        self.lego_coords = [[0,0], [1,1]] # etc... etc..

    def main(self, isMain=False):
        """Camera Display Example
        Cognex Hand Camera Ranges
            - exposure: [0.01-100]
            - gain: [0-255]
        Head Camera Ranges:
            - exposure: [0-100], -1 for auto-exposure
            - gain: [0-79], -1 for auto-gain
        """
        rp = intera_interface.RobotParams()
        valid_cameras = rp.get_camera_names()
        if not valid_cameras:
            rp.log_message(("Cannot detect any camera_config"
                " parameters on this robot. Exiting."), "ERROR")
            return
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
        parser.add_argument(
            '-c', '--camera', type=str, default="right_hand_camera",
            choices=valid_cameras, help='Setup Camera Name for Camera Display')
        parser.add_argument(
            '-r', '--raw', action='store_true',
            help='Specify use of the raw image (unrectified) topic')
        parser.add_argument(
            '-e', '--edge', action='store_true',
            help='Streaming the Canny edge detection image')
        parser.add_argument(
            '-g', '--gain', type=int,
            help='Set gain for camera (-1 = auto)')
        parser.add_argument(
            '-x', '--exposure', type=float,
            help='Set exposure for camera (-1 = auto)')
        args = parser.parse_args(rospy.myargv()[1:])


        if isMain==True:
            print("Initializing node... ")
            rospy.init_node('camera_display', anonymous=True)
        cameras = intera_interface.Cameras()
        if not cameras.verify_camera_exists(args.camera):
            rospy.logerr("Could not detect the specified camera, exiting the example.")
            return
        rospy.loginfo("Opening camera '{0}'...".format(args.camera))
        cameras.start_streaming(args.camera)
        rectify_image = not args.raw
        use_canny_edge = args.edge
        cameras.set_callback(args.camera, self.show_image_callback, rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

        print("ARGS rectify TYPE", type(rectify_image))
        # show_image_callback(rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

        # optionally set gain and exposure parameters
        if args.gain is not None:
            if cameras.set_gain(args.camera, args.gain):
                rospy.loginfo("Gain set to: {0}".format(cameras.get_gain(args.camera)))

        if args.exposure is not None:
            if cameras.set_exposure(args.camera, args.exposure):
                rospy.loginfo("Exposure set to: {0}".format(cameras.get_exposure(args.camera)))

        rospy.on_shutdown(self.clean_shutdown)
        rospy.loginfo("Camera_display node running. Ctrl-c to quit")
        # rospy.spin()

    def clean_shutdown(self):
            print("Shutting down camera_display node.")
            cv2.destroyAllWindows()
            return


if __name__ == '__main__':
    obj = ArmCamera()