#!/usr/bin/env python

from rospy import Publisher
from rospy import ROSInterruptException
from rospy import init_node
from rospy import Rate
from rospy import is_shutdown
from geometry_msgs.msg import Twist


def print_instruction():
    print('Reading from keyboard')
    print('---------------------------')
    print('To move the turtle use keys:')
    print('a - to turn left')
    print('d - to turn right')
    print('w - to go forward')
    print('s - to go backwards')


def get_move():
    key = raw_input('')
    twist = Twist()
    if key == 'w':
        twist.linear.x = 2.0
        return twist
    if key == 's':
        twist.linear.x = -2.0
        return twist
    if key == 'a':
        twist.angular.z = 2.0
        return twist
    if key == 'd':
        twist.angular.z = -2.0
        return twist
    return None


def do_sample():
    pub = Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    init_node('sample_talker')
    rate = Rate(10)
    print_instruction()
    while not is_shutdown():
        move = get_move()
        if move is not None:
            pub.publish(move)
        rate.sleep()


if __name__ == '__main__':
    try:
        do_sample()
    except ROSInterruptException:
        pass
