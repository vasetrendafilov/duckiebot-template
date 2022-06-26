#!/usr/bin/env python3
import os
import rospy
import numpy as np
from my_package import Duckiebot

def process_image(img):
    dec_img = duckie.decode_image(img)
    duckie.save_image(dec_img)

if __name__ == '__main__':
    # create the node
    duckie = Duckiebot(node_name='my_publisher_node')
    # connect camera
    duckie.connect_camera(process_image)
    # test wheels
    duckie.save_image('slikiw',np.zeros((153, 231, 3)))
    duckie.publish_wheel_cmd(0.2, 0.2)
    # keep spinning
    rospy.spin()