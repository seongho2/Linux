import rospy
import sys
import tty
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import String

velocity = 0
steering = 0
breakcontrol = 1
gear = 0
MAX_Velocity = 255
MIN_Velocity = -255
MAX_R_ANGLE = 50
MAX_L_ANGLE = -50

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def getkey():
    fd = sys.stdin.fileno()
    original_attributes = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
    return ch

def teleop():
    global velocity, steering, breakcontrol, gear
    rospy.init_node('teleop', anonymous=True)
    rate = rospy.Rate(10)
    status = 0
    while not rospy.is_shutdown():
        key = getkey()
        if key == 'w':
            velocity = velocity + 5
            status += 1
        elif key == 'x':
            velocity = 0
            steering = 0
            status += 1
        elif key == 'd':
            steering = steering + 1
            status += 1
        elif key == 'a':
            steering = steering - 1
            status += 1
        elif key == 's':
            velocity = velocity - 5
            status += 1
        else:
            if (key == '\x03'):
                break
        pubmsg = Twist()
        if velocity >= MAX_Velocity:
            velocity = MAX_Velocity
        if velocity <= MIN_Velocity:
            velocity = MIN_Velocity
        if steering >= MAX_R_ANGLE:
            steering = MAX_R_ANGLE
        if steering <= MAX_L_ANGLE:
            steering = MAX_L_ANGLE

        pubmsg.linear.x = velocity
        pubmsg.angular.z = steering
        publisher.publish(pubmsg)
        print('car speed: ' + str(velocity))
        print('steering angle: ' + str(steering))
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: 
        pass

