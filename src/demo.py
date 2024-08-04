import rospy
from hand_v3.msg import RawAngle
import time

rospy.init_node("demo")
kondo_angle_pub = rospy.Publisher("/kondo/command_angle", RawAngle, queue_size=1)
kondo_speed_pub = rospy.Publisher("/kondo/command_speed", RawAngle, queue_size=1)
futaba_angle_pub = rospy.Publisher("/futaba/command_angle", RawAngle, queue_size=1)

def set_speed(speed):
    kondo_msg = RawAngle()
    kondo_msg.angle = [speed for x in range(4)]
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

def beckoning():
    # kondo_msg = RawAngle()
    # kondo_msg.angle = [7000]
    # kondo_msg.id = [3]
    # kondo_msg.length = 1
    # kondo_angle_pub.publish(kondo_msg)
    # time.sleep(0.6)

    for i in range(2):
        kondo_msg = RawAngle()
        kondo_msg.angle = [5500]
        kondo_msg.id = [1]
        kondo_msg.length = 1
        kondo_angle_pub.publish(kondo_msg)
        time.sleep(0.8)

        kondo_msg = RawAngle()
        kondo_msg.angle = [6800]
        kondo_msg.id = [1]
        kondo_msg.length = 1
        kondo_angle_pub.publish(kondo_msg)
        time.sleep(0.8)

def beckoning_demo():
    init_pose()
    beckoning()
    init_pose()

def init_to_peace_pose():
    kondo_msg = RawAngle()
    kondo_msg.angle = [5500]
    kondo_msg.id = [0]
    kondo_msg.length = 1
    kondo_angle_pub.publish(kondo_msg)

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
        futaba_msg.angle = [60-i,-50+i]
        futaba_msg.id = [1,2]
        futaba_msg.length = 2
        futaba_angle_pub.publish(futaba_msg)
        i += 1
        time.sleep(0.01)

def peace_to_init_pose():
    kondo_msg = RawAngle()
    kondo_msg.angle = [9500]
    kondo_msg.id = [0]
    kondo_msg.length = 1
    kondo_angle_pub.publish(kondo_msg)

    for i in range(90):
        futaba_msg = RawAngle()
        futaba_msg.angle = [-30+i,40-i]
        futaba_msg.id = [1,2,]
        futaba_msg.length = 2
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

def peace_demo():
    init_to_peace_pose()
    rospy.sleep(1.5)
    peace_to_init_pose()

def gripping_pen():
    for i in range(85):
        futaba_msg = RawAngle()
        futaba_msg.angle = [60-i,-50+i,60-i,-60]
        futaba_msg.id = [1,2,4,3]
        futaba_msg.length = 4
        futaba_angle_pub.publish(futaba_msg)
        i += 1
        time.sleep(0.01)

    time.sleep(2)

    i = -60
    while i < -20:
        futaba_msg = RawAngle()
        futaba_msg.angle = [i]
        futaba_msg.id = [0]
        futaba_msg.length = 1
        futaba_angle_pub.publish(futaba_msg)
        time.sleep(0.01)
        i += 1

    i = -60
    while i < 20:
        futaba_msg = RawAngle()
        futaba_msg.angle = [i]
        futaba_msg.id = [5]
        futaba_msg.length = 1
        futaba_angle_pub.publish(futaba_msg)
        time.sleep(0.01)
        i += 1

    for i in range(70):
        futaba_msg = RawAngle()
        futaba_msg.angle = [-60+i]
        futaba_msg.id = [3]
        futaba_msg.length = 1
        futaba_angle_pub.publish(futaba_msg)
        i += 1
        time.sleep(0.01)

def gripping_pen_demo():
    init_pose()
    gripping_pen()
    kondo_msg = RawAngle()
    kondo_msg.angle = [5500, 6500]
    kondo_msg.id = [1,3]
    kondo_msg.length = 2
    kondo_angle_pub.publish(kondo_msg)
    rospy.sleep(1)
    init_arm_pose()
    init_hand_pose()

def init_to_rock_pose():
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
        futaba_msg.angle = [60-i,-50+i,60-i,-60+i]
        futaba_msg.id = [1,2,4,3]
        futaba_msg.length = 4
        futaba_angle_pub.publish(futaba_msg)
        i += 1
        time.sleep(0.01)

def jyanken():
    kondo_msg = RawAngle()
    kondo_msg.angle = [5000]
    kondo_msg.id = [2]
    kondo_msg.length = 1
    kondo_angle_pub.publish(kondo_msg)
    init_to_rock_pose()

    for i in range(3):
        kondo_msg = RawAngle()
        kondo_msg.angle = [7000]
        kondo_msg.id = [3]
        kondo_msg.length = 1
        kondo_angle_pub.publish(kondo_msg)
        time.sleep(0.8)

        kondo_msg = RawAngle()
        kondo_msg.angle = [7500]
        kondo_msg.id = [3]
        kondo_msg.length = 1
        kondo_angle_pub.publish(kondo_msg)
        time.sleep(0.8)
