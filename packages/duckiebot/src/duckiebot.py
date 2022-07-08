#!/usr/bin/env python3
import os
import rospy
from my_package import Duckiebot

def process_image(img):
    # always decode image
    dec_img = duckie.decode_image(img)
    duckie.save_image('turn1',dec_img)

if __name__ == '__main__':
    # create the node
    duckie = Duckiebot(node_name='my_node')
    # connect camera
    duckie.connect_camera(process_image)
    # test wheels
    duckie.publish_wheel_cmd(0.2, 0.2)
    # keep spinning
    rospy.spin()