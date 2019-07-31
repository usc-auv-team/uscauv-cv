import sys,tty,termios
from geometry_msgs.msg import Vector3, Vector3Stamped
import ros_utils as ru

class _Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(3)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def get(current_yaw):
    ret = True
    forwards_power = 0

    inkey = _Getch()
    while(1):
        k=inkey()
        if k!='':break
    if k=='\x1b[A':
        # Make forwards power = 0.5
        forwards_power = 0.5
        print "Forwards"
    elif k=='\x1b[B':
        # Make forwards power = -0.5
        forwards_power = -0.5
        print "Back"
    elif k=='\x1b[C':
        # Increase yaw angle by 1
        current_yaw += 1
        print "Right"
    elif k=='\x1b[D':
        # Decrease yaw angle by 1
        current_yaw -= 1
        print "Left"
    elif k=='q'
        ret = False

    # Forwards power should already be 0
    # if k!='\x1b[A' or k!='\x1b[B':
    #      # make forwards power = 0

    ru.motors_client(current_yaw, forwards_power)

    return ret

def main():

    current_yaw = 0
    desired_yaw = 0

    try:
        rospy.init_node('keyboard_controls',anonymous = True)
        rospy.Subscriber('ngimu/euler', Vector3Stamped, get_imuangle)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")

    loop = True
    while loop:
        data = rospy.wait_for_message('ngimu/euler', Vector3Stamped)
        current_yaw = data.vector.z
        loop = get(current_yaw)

if __name__=='__main__':
        main()
