import rospy
from sensor_msgs.msg import Image 

import cv2
from cv_bridge import CvBridge, CvBridgeError

cv_bridge_inst=CvBridge()

def callback_foo(x):
    #print(len(x.data))
    try:
        cv_image=cv_bridge_inst.imgmsg_to_cv2(x,"bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    blur=cv2.blur(cv_image,(9,9) )
    gr=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("cam_2", cv_image)
    cv2.imshow("blurred", blur)
    cv2.imshow("gray ", gr)


    cv2.waitKey(3)





def main():
    rospy.init_node("test_camera_node")
    sub= rospy.Subscriber("/cam_2/camera/image",Image,callback_foo)
    rospy.spin()





if __name__=="__main__":
    main()