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


q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

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

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

modeBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if _name_ == "_main_":
    
               
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

    try:
        print (msg)
        print (vels(speed, turn))
        # take off
        key = getKey("1")
        if key == '1':
            print('Despegue y subida momentánea')
            empty = Empty()
            pub_takeoff.publish(empty)
        # Elevar el dron 5 veces
        for i in range(5):
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 1.0  # Ajusta la velocidad de elevación aquí
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            pub.publish(twist)

            # Espera un tiempo para elevarse
            time.sleep(2)  # Ajusta el tiempo de espera aquí

            # Detener la elevación
            twist.linear.z = 0
            pub.publish(twist)

        while(1):
            key = getKey()
            # print(key)
            # print(ord(key))
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            #elif key in speedBindings.keys():
            #speed = speed * speedBindings[key][0]
            #turn = turn * speedBindings[key][1]
            #print vels(speed,turn)
            #     if (status == 14):
            #           print msg
            #     status = (status + 1) % 15
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

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

    except e:
        print (e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)