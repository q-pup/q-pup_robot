import rospy
from rosgraph_msgs.msg import Clock
import time

def clock_server():
    pub = rospy.Publisher('/clock', Clock, queue_size=10)
    rospy.init_node('clock_server', anonymous=True)
    sim_speed_multiplier = 1/3
    sim_clock = Clock()
    zero_time = time.time()

    while not rospy.is_shutdown():
        sim_clock.clock = rospy.Time.from_sec(sim_speed_multiplier*(time.time() - zero_time))
        pub.publish(sim_clock)
        time.sleep(1/1000)  # 1kHz ish


if __name__ == '__main__':
    clock_server()
