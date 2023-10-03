#!/usr/bin/env python
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty

import sys
import select
import termios
import tty
import time

msg = """
Mapa de teclas:
---------------------------
   q    w   e
   a        d
   z    s   c
---------------------------
+ : sube (+z)
- : baja (-z)
---------------------------
Girar el dron (rot z),
utilizar Shift + A o Shift + D:

Izquierda: A
Derecha: D



CTRL-C para salir.
"""

moveBindings = {
    'w': (1, 0, 0, 0),          # Frente
    'a': (0, 1, 0, 0),          # Izquierda
    'd': (0, -1, 0, 0),         # Derecha
    's': (-1, 0, 0, 0),         # Atrás
    'q': (1, 1, 0, 0),          # Diagonal izquierda frente
    'e': (1, -1, 0, 0),         # Diagonal derecha frente
    'c': (-1, -1, 0, 0),        # Diagonal derecha atrás
    'z': (-1, 1, 0, 0),         # Diagonal izquierda atrás
    '+':(0,0,1,0),              # Sube
    '-':(0,0,-1,0),             # Baja
    'A':(0,0,0,1),              # Giro a la izquierda
    'D':(0,0,0,-1),             # Giro a la derecha
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
    pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
    pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)
    

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    twist = Twist()
    
    twist.linear.z = 1
    pub.publish(twist)

    try:
        print (msg)
        empty = Empty()
        rospy.sleep(1)
        print('volando')
        pub_takeoff.publish(empty)
        rospy.sleep(1)
        for i in range(5):
            twist.linear.z = 1
            rospy.sleep(1)
            pub.publish(twist)
        

        while(1):
            key = getKey()

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

            elif key == '1':
                print('Despegue y subida momentánea')
                empty = Empty()
                pub_takeoff.publish(empty)
                

            elif key == '2':
                print('Aterrizaje')
                empty = Empty()
                pub_land.publish(empty)

            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
           
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

    finally:
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)