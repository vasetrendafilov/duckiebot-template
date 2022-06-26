import os
import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, Twist2DStamped
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class Duckiebot(DTROS):
    def __init__(self,node_name):
        # initialize the DTROS parent class
        super(Duckiebot, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Variables
        self.img_count = 0
        self.right_encoder_tics = 0
        self.left_encoder_tics = 0

        # Setup Parameters  treba da se namestat
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        #self.omega_gain = self.setupParam("~steer_gain", 8.3)
        #self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        #self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        #self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_wheel = rospy.Publisher('/patka1/wheels_driver_node/wheels_cmd',WheelsCmdStamped, queue_size=1)
        self.pub_car = rospy.Publisher('/patka1/car_cmd_switch_node/cmd',Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_right_encoder = rospy.Subscriber("/patka1/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_encoder, queue_size=1)
        self.sub_left_encoder =  rospy.Subscriber("/patka1/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_encoder, queue_size=1)
    
    def right_encoder(self,data):
        self.right_encoder_tics = data.data
        rospy.loginfo("I heard that the right wheel is %s tics and resolution of %s", str(data.data),str(data.resolution))

    def left_encoder(self,data):
        self.left_encoder_tics = data.data
        rospy.loginfo("I heard that the left wheel is %s tics and resolution of %s", str(data.data),str(data.resolution))

    def connect_camera(self,callback):
        self.sub_image = rospy.Subscriber("/patka1/camera_node/image/compressed", CompressedImage, callback, queue_size=1)

    def decode_image(self,image):
        bridge = CvBridge()
        try:
            self.img_count +=1
            decoded_img = bridge.compressed_imgmsg_to_cv2(image)
        except CvBridgeError as e:
            print(e)
            return None
        return decoded_img

    def save_image(self,location,image):
        # moze da se napravi da zacivuva sekoi 5 sliki takva rab
        if not os.path.isdir(f"/data/{location}/"):
            os.mkdir(f"/data/{location}/")
        cv2.imwrite(f"/data/{location}/"+str(self.img_count)+".jpg",image)
        
    def publish_car_cmd(self,velocity,omega):
        cmd_msg = Twist2DStamped()
        cmd_msg.v = velocity
        cmd_msg.omega = omega
        self.pub_car.publish(cmd_msg)
        rospy.loginfo("Published speed for car (%s) and direction (%s)", str(cmd_msg.v), str(cmd_msg.omega))

    def publish_wheel_cmd(self,velocity_left,velocity_right):
        cmd_msg = WheelsCmdStamped()
        cmd_msg.vel_left = velocity_left
        cmd_msg.vel_right = velocity_right
        self.pub_wheel.publish(cmd_msg)
        rospy.loginfo("Published speed for left (%s) and right (%s) wheel", str(cmd_msg.vel_left), str(cmd_msg.vel_right))
    
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value)
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
