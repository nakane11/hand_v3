import rospy
from hand_v3.msg import RawAngle
import time

rospy.init_node("demo")
kondo_angle_pub = rospy.Publisher("/kondo/command_angle", RawAngle, queue_size=1)
kondo_speed_pub = rospy.Publisher("/kondo/command_speed", RawAngle, queue_size=1)
futaba_angle_pub = rospy.Publisher("/futaba/command_angle", RawAngle, queue_size=1)

def set_speed():
    kondo_msg = RawAngle()
    kondo_msg.angle = [50,50,50,50]
    kondo_msg.id = [0,1,2,3]
    kondo_msg.length = 4
    kondo_speed_pub.publish(kondo_msg)

def init_hand_pose():
    kondo_msg = RawAngle()
    kondo_msg.angle = [9500]
    kondo_msg.id = [0]
    kondo_msg.length = 1
    kondo_angle_pub.publish(kondo_msg)
    
    futaba_msg = RawAngle()
    futaba_msg.angle = [60,-50,60,-60,-60,-60]
    futaba_msg.id = [1,2,4,3,0,5]
    futaba_msg.length = 6
    futaba_angle_pub.publish(futaba_msg)

def init_arm_pose():
    kondo_msg = RawAngle()
    kondo_msg.angle = [7500,7500,7500]
    kondo_msg.id = [1,2,3]
    kondo_msg.length = 3
    kondo_angle_pub.publish(kondo_msg)

def init_pose():
    init_hand_pose()
    init_arm_pose()
    
def init_to_pointing_pose():
    i = -60
    while i < 20:
        futaba_msg = RawAngle()
        futaba_msg.angle = [i,i]
        futaba_msg.id = [0,5]
        futaba_msg.length = 2
        futaba_angle_pub.publish(futaba_msg)
        time.sleep(0.01)
        i += 1
        
    for i in range(90):
        futaba_msg = RawAngle()
        futaba_msg.angle = [60-i,-50+i,60-i,-60]
        futaba_msg.id = [1,2,4,3]
        futaba_msg.length = 4
        futaba_angle_pub.publish(futaba_msg)
        i += 1
        time.sleep(0.01)

def pointing_to_init_pose():
    for i in range(90):
        futaba_msg = RawAngle()
        futaba_msg.angle = [-30+i,40-i,-30+i,-60]
        futaba_msg.id = [1,2,4,3]
        futaba_msg.length = 4
        futaba_angle_pub.publish(futaba_msg)
        i += 1
        time.sleep(0.01)
        
    i = 20
    while i >= -60:
        futaba_msg = RawAngle()
        futaba_msg.angle = [i,i]
        futaba_msg.id = [0,5]
        futaba_msg.length = 2
        futaba_angle_pub.publish(futaba_msg)
        time.sleep(0.01)
        i -= 1

def pointing_left():
    kondo_msg = RawAngle()
    kondo_msg.angle = [5500,5000]
    kondo_msg.id = [1,2]
    kondo_msg.length = 2
    kondo_angle_pub.publish(kondo_msg)

    time.sleep(1.0)

    kondo_msg = RawAngle()
    kondo_msg.angle = [7500,7500]
    kondo_msg.id = [1,2]
    kondo_msg.length = 2
    kondo_angle_pub.publish(kondo_msg)

def pointing_demo():
    init_pose()
    init_to_pointing_pose()
    pointing_left()
    init_arm_pose()
    time.sleep(0.2)
    pointing_to_init_pose()
