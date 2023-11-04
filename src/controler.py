import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
from  nav_msgs.msg import Odometry
from geometry_msgs.msg import  Point
from sensor_msgs.msg import Image 
import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf.transformations as ttt 
cv_bridge_inst=CvBridge()

ALTITUDE_VALUE=3
kz=5
bz=10
ks=0.01*20
bs=0.005*10
ky=0.004
by=0.0005


class controller:

    def __init__(self):
        rospy.init_node("controller_node")
        rospy.Subscriber("/ground_truth/state",Odometry, self.state_callback)
        rospy.Subscriber("/cam_1/camera/image",Image, self.camera_callback)
        rospy.on_shutdown(self.stop_robot)

        self.cmd_pub=rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        self.position=Point()
        self.twist=Twist()
        self.omega_error = 0
        self.omega_error_prev=0
        self.y_error=0
        self.y_error_prev=0
    
    def steering_ctrl(self):
        u_s=ks*self.omega_error -bs*(self.omega_error-self.omega_error_prev)/(1.0/50.0)
        self.omega_error_prev=self.omega_error
        return u_s

    def offset_ctrl(self):
        u_y=ky*self.y_error -by*(self.y_error-self.y_error_prev)/(1.0/50.0)
        self.y_error_prev=self.y_error
        return u_y 

    def state_callback(self, msg):
        self.position=msg.pose.pose.position
        self.twist=msg.twist.twist

    def camera_callback(self,msg):
        try:
            cv_image = cv_bridge_inst.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(cv_image, (160, 0), (160, 240), (0, 123, 0), 1)
        cv2.line(cv_image, (0, 120), (320, 120), (0, 123, 0), 1)

        # "steering" conrol
        top_points = np.where(mask[10] >= 10)
        mid_points = np.where(mask[msg.height / 2] >= 10)
        if  (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
            top_line_point = int(np.average(top_points))
            mid_line_point = int(np.average(mid_points))
            self.omega_error = top_line_point - mid_line_point
            
            cv2.circle(cv_image, (top_line_point, 10), 5, (0,0,255), 1)
            cv2.circle(cv_image, (mid_line_point, int(msg.height/2)), 5, (0,0,255), 1)
            cv2.line(cv_image, (mid_line_point, int(msg.height/2)), (top_line_point, 10), (0, 0, 255), 3)

        # y-offset control
        __, cy_list = np.where(mask >= 10)
        if not np.isnan(np.average(cy_list)):
            cy = int(np.average(cy_list))
            self.y_error = msg.width / 2 - cy
            
            cv2.circle(cv_image, (cy, int(msg.height/2)), 7, (0,255,0), 1)
            cv2.line(cv_image, (160, 120), (cy, int(msg.height/2)), (0, 255, 0), 3)

        self.show_image(cv_image)

    def show_image(self, img, title='Camera 1'):
        cv2.imshow(title, img)
        cv2.waitKey(3)

    def enable_motors(self):
        rospy.wait_for_service("/enable_motors")
        foo2call = rospy.ServiceProxy("/enable_motors",EnableMotors)
        if foo2call(True):
            print(" motor working")


    def spin(self):
        self.enable_motors()
        try:
            rate=rospy.Rate(50.0)
            while not rospy.is_shutdown():
                u_z=self.takeoff_ctrl(ALTITUDE_VALUE)
                u_s=self.steering_ctrl()
                u_y=self.offset_ctrl()

                cmd_msg=Twist()
                cmd_msg.linear.z=u_z
                cmd_msg.linear.y=u_y
                cmd_msg.linear.x=1.5
                cmd_msg.angular.z=-u_s
                self.cmd_pub.publish(cmd_msg)
                
        except :
            self.stop_robot()
        rate.sleep()


    def stop_robot(self):
        cmd_pub=rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        cmd_msg=Twist()
        cmd_msg.linear.x=0
        cmd_msg.linear.y=0
        cmd_msg.linear.z=0
        cmd_msg.angular.z=0
        cmd_pub.publish(cmd_msg)

        
#    def __del__(self):
 #       self.stop_robot()

    def takeoff_ctrl(self,z_des):
        return kz*( z_des-self.position.z)- bz*self.twist.linear.z
         



def main():
    ct=controller()
    ct.spin()
    




if __name__=="__main__":
    main()