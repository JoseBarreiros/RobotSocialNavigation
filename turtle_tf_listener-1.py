#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import numpy as np

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)

     #init variables
    
    G=0.1 #atraction rate to the goal 
    R=3 #rate of repulsion to unfamiliar turtles
    Xg=np.array([10,10]) #x,y
    ar=np.array([0,0])
    ag=np.array([0,0])
    T=0.5 #simulation period


    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
            (Xi,rot_i) = listener.lookupTransform('/turtle1', '/turtle1', rospy.Time(0))

            trans_np=np.array([trans[0],trans[1]])
            Xn=np.array(Xi)  #position of turtle 1
            print(Xn)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #ag=G*np.substract(Xg,Xn)/np.linalg.norm(np.substract(Xg,Xn))
        ag=G*np.array([5,5])
        distance=math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        if (distance<=1):
                ar=-R*trans_np/distance
        else:
                ar=np.array([0,0])
				
        aa=ag+ar
        #V=aa*T
        #print(V)
        linear=math.sqrt(aa[0] ** 2 + aa[1] ** 2)
	angular = math.atan2(aa[1], aa[0])
	cmd = geometry_msgs.msg.Twist()
	cmd.linear.x = linear
	cmd.angular.z = angular
	turtle_vel.publish(cmd)
        rate.sleep()
