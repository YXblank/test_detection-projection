#!/usr/bin/env python

import rospy
import roslib
import tf
import sys
import math
import numpy as np
import struct
import copy
from visualization_msgs.msg import Marker
from custom_msgs.msg import WorldObject
from custom_msgs.msg import ObjectList
from rtabmap_ros.msg import MapGraph

from filtered_instances import FilteredInstances

# Param namespace: hardcoded here in order to use a single param file, otherwise would need 2 yaml param files. 
param_ns = "/object_positioner"

# Classes param [in]
classes_param = param_ns + "/classes"

# TOPICS [in]
object_list_topic_raw = '/objects_raw/list'
graph_list = '/rtabmap/mapGraph'

# TOPICS [out]
objects_topic_filtered = '/objects_filtered'

# Perform graph node update 
doGraphUpdate = False
printCovariances = False
printPositions = False
printMeanError = True

# FILTER 
process_cov = 0.3
meas_cov = 5
min_obs = 6.0

# Classes 
cabins = None 
passageways = None


# Association threshold 
cabin_radius = 3
passageway_radius = 4


# ground truths

# DCC dataset

cabin_gt = [(-3.668, 4.993), (-2.594, 4.192), (-0.181, -5.362), (-1.544, -3.578), (1.686, -3.738), (0.711, -5.175), (-0.579, -2.481), (11.019, 9.995), (11.988, 11.240), (17.875, 20.793), (17.020, 19.669), (24.580, 29.927), (22.345, 26.843), (21.484, 25.707), (32.509, 37.543), (31.188, 39.274), (30.447, 39.114), (19.669, 40.091) , (20.922, 39.260)]
passageway_gt = [(13.155, 15.224)]



# Debug
markers_topic = '/markers'

def object_list_callback(object_list):
    cabin_list = []
    passageway_list = []
   
    for obj in object_list.objects:
        if obj.objClass == 'cabin':
            cabin_list.append((obj.x,obj.y, obj.angle)) 

        elif obj.objClass == 'passageway':
            passageway_list.append((obj.x,obj.y, obj.angle))

    

    cabins.addMeasurementList(cabin_list)
    passageways.addMeasurementList(passageway_list)
    

def graph_list_callback(graph_list):
    global cabins, passageways
    cabins.updateGraphList(graph_list.poses, graph_list.posesId)
    passageways.updateGraphList(graph_list.poses, graph_list.posesId)
    
def getTextMarker(label, x, y, height, namespace, id, frame, size, R, G, B, lifeTime):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()		

    # Frame (map)
    marker.header.frame_id = frame

    # Object type
    marker.ns = namespace+"_label"

    # Marker identifier
    marker.id = id

    marker.text = label

    # Text
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    # Size
    marker.scale.z = size

    # Position
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = height

    # Color
    marker.color.a = 1.0 
    marker.color.r = R
    marker.color.g = G
    marker.color.b = B

    # Lifetime
    marker.lifetime = rospy.Duration(lifeTime)

    return marker

def getMarker(x, y, z, angle, namespace, id, frame, size_x=0.4, size_y=0.4, size_z=0.4, R=1.0,G=0.0,B=0.0, lifeTime=5.0):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()		

    # Frame (map)
    marker.header.frame_id = frame

    # Object type
    marker.ns = namespace

    # Marker identifier
    marker.id = id

    # Sphere
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # Size
    marker.scale.x = size_x
    marker.scale.y = size_y
    marker.scale.z = size_z

    # Position
    q = tf.transformations.quaternion_from_euler(0, 0, angle)
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    # Color
    marker.color.a = 1.0 
    marker.color.r = R
    marker.color.g = G
    marker.color.b = B

    # Lifetime
    marker.lifetime = rospy.Duration(lifeTime)

    return marker

def getMarkerArrow(x, y, angle, namespace, id, frame, size=0.4, R=1.0,G=0.0,B=0.0, lifeTime=5.0):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()		

    # Frame (map)
    marker.header.frame_id = frame

    # Object type
    marker.ns = namespace

    # Marker identifier
    marker.id = id

    # Sphere
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    q = tf.transformations.quaternion_from_euler(0, 0, angle)

    # Position
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.3
    #marker.pose.position.z = 0.6
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    # Size
    marker.scale.x = size
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # Color
    marker.color.a = 1.0 
    marker.color.r = R
    marker.color.g = G
    marker.color.b = B

    # Lifetime
    marker.lifetime = rospy.Duration(lifeTime)

    return marker

def main(args):
    
    global object_list_topic_raw, object_list_topic_raw, process_cov, meas_cov, doGraphUpdate
    global cabins, passageways
    global cabin_radius, passageway_radius
    global cabin_gt, passageway_gt
    global min_obs

    # Initialize node
    rospy.init_node('object_marker', anonymous=True)
    rate = rospy.Rate(5) 

    rospy.Subscriber(object_list_topic_raw, ObjectList, object_list_callback)
    if doGraphUpdate:
        rospy.Subscriber(graph_list, MapGraph, graph_list_callback)

    obj_pub = rospy.Publisher(objects_topic_filtered, WorldObject, queue_size=10)
    marker_pub = rospy.Publisher(markers_topic, Marker, queue_size=10)

    # Object instance lists
    cabins = FilteredInstances('cabin', cabin_radius, process_cov, meas_cov, min_obs, cabin_gt)
    passageways = FilteredInstances('passageway', passageway_radius, process_cov, meas_cov, min_obs, passageway_gt)

    while not rospy.is_shutdown(): 
        
        life_time = 0
        # Publish doors
        for i in range(len(cabins.instances)):
            pred = cabins.predictions[i]
            obj_filtered = WorldObject()
            obj_filtered.objClass = 'cabin'
            obj_filtered.x = pred[0]
            obj_filtered.y = pred[1]
            obj_filtered.angle = cabins.angles[i]
            obj_filtered.prob = float(i)
            
            if cabins.observations[i] > min_obs:
                obj_pub.publish(obj_filtered)

                class_name = 'cabin'
                height = 0.4
                frame = 'map'
                size_x = 0.05
                size_y = 0.6
                size_z = 1.8
                R = 0.5
                G = 0.1 
                B = 0.0

                # Publish marker
                marker = getMarker(obj_filtered.x-10, obj_filtered.y-10, height/20, obj_filtered.angle, class_name, i, frame, size_x, size_y, size_z, R, G, B, life_time)
                marker_pub.publish(marker)

                text = getTextMarker(class_name, obj_filtered.x, obj_filtered.y, height+size_z/2.0 + 0.5, class_name, i, frame, 0.3, R, G, B, life_time)
                marker_pub.publish(text)

        # Publish benches
        for i in range(len(passageways.instances)):
            pred = passageways.predictions[i]
            obj_filtered = WorldObject()
            obj_filtered.objClass = 'passageway'
            obj_filtered.x = pred[0]
            obj_filtered.y = pred[1]
            obj_filtered.angle = benches.angles[i]
            obj_filtered.prob = float(i)
            
            if benches.observations[i] > min_obs:
                obj_pub.publish(obj_filtered)

                class_name = 'passageway'
                height = 0.4
                frame = 'map'
                size_x = 0.4
                size_y = 0.4
                size_z = 0.5
                R = 0.0
                G = 0.1 
                B = 0.8

                # Publish marker
                marker = getMarker(obj_filtered.x-10, obj_filtered.y, height, obj_filtered.angle, class_name, i, frame, size_x, size_y, size_z, R, G, B, life_time)
                marker_pub.publish(marker)

                text = getTextMarker(class_name, obj_filtered.x, obj_filtered.y, height+size_z/2 , class_name, i, frame, 0.3, R, G, B, life_time)
		#text = getTextMarker(class_name, obj_filtered.x, obj_filtered.y, height+size_z/2.0 + 0.5, class_name, i, frame, 0.3, R, G, B, life_time)		
                marker_pub.publish(text)

        # Publish trashes
      
            
        rate.sleep()

if __name__ == '__main__':
    try: 
        # print rospy.get_param(classes_param)[0]
        main(sys.argv)
    except rospy.exceptions.ROSInterruptException:
        print 'SHUTTING DOWN'
        
        if(printCovariances):
            num, vx2, vy2, cxy = cabins.getMeanCovariance()
            print 'cabins: vx2 = '+vx2+', vy2 = '+vy2+', cxy = '+cxy+' ['+num+' instances]'

            num, vx2, vy2, cxy = passageways.getMeanCovariance()
            print 'passageways: vx2 = '+vx2+', vy2 = '+vy2+', cxy = '+cxy+' ['+num+' instances]'

          
            print '\n'

        if(printMeanError):
            number, error, fpositives, fnegatives = cabins.getMeanError()
            print 'cabins ' + number +' : error = '+error+', fpositives = '+fpositives+', fnegatives = '+fnegatives

            number, error, fpositives, fnegatives = passageways.getMeanError()
            print 'passageways ' + number +' : error = '+error+', fpositives = '+fpositives+', fnegatives = '+fnegatives

          
            print '\n'
            
        if(printPositions):

            for i in range(len(cabins.instances)):
                pred = cabins.predictions[i]
                x = pred[0]
                y = pred[1]
                if cabins.observations[i] > min_obs:
                    print 'cabin '+str(i+1)+' : '+str(x) + ' '+str(y) 
            print '\n'

            for i in range(len(passageways.instances)):
                pred = passageways.predictions[i]
                x = pred[0]
                y = pred[1]
                if passageways.observations[i] > min_obs:
                    print 'passageway '+str(i+1)+' : '+str(x) + ' '+str(y) 
            print '\n'

           
