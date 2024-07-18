#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import MeanShift, estimate_bandwidth
from uav_frontier_exploration_3d.msg import ClusterInfo


num_prev_markers = 0

def callback(pose_array):
    global num_prev_markers

    # Estraggo i punti dal messaggio PoseArray:
    # il topic geometry_msgs::PoseArray contiene un array di pose, dove c'è la proprietà "poses" che è un messaggio di tipo geometry_msgs::Pose
    points = []
    for pose in pose_array.poses:
        points.append([pose.position.x, pose.position.y, pose.position.z]) 

    # Converto in un array numPy
    points = np.array(points)

    # Clustering Mean-Shift
    rospy.loginfo(f"quantile: {quantile}")
    bandwidth = estimate_bandwidth(points, quantile = quantile)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(points)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_

    labels_unique = np.unique(labels)
    n_clusters_ = len(labels_unique)
    print("number of estimated clusters : %d" % n_clusters_)

    # Inizializzo il msg custom 
    cluster_info_msg = ClusterInfo()
    cluster_info_msg.clst_centers = [] # attributi msg custom
    cluster_info_msg.clst_radius = []  # attributi msg custom

    # Marker dei cluster  per Rviz
    marker_array = MarkerArray()
    for i, center in enumerate(cluster_centers):
        # Centers and radius of clustered spheres  
        cluster_points = points[labels == i]
        radius_cluster = np.max(np.linalg.norm(cluster_points - center, axis=1))  # max distanza tra centro del cluster e i punti, che viene utilizzato come raggio della sfera.
                                                                                  # (np.linalg.norm calcola la norma di ogni vettore spostamento. axis=1 indica che la norma deve essere calcolata lungo l'asse colonne)
        # print(f"Cluster {i}: Center = {center}, Radius = {radius_cluster}")
        marker = Marker()
        marker.id = i
        marker.ns = marker_ns
        marker.header.frame_id = frame
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius_cluster*2 # scale è riferito al diametro!
        marker.scale.y = radius_cluster*2
        marker.scale.z = radius_cluster*2
        marker.color.r = 0
        marker.color.g = 255
        marker.color.b = 255
        marker.color.a = 0.3
        marker_array.markers.append(marker)

        # Creo il topic custom msg (Point centers, float radius)
        point = Point()
        point.x = center[0]
        point.y = center[1]
        point.z = center[2]
        cluster_info_msg.clst_centers.append(point)
        cluster_info_msg.clst_radius.append(radius_cluster)


    num_current_markers = len(cluster_centers)
    if num_prev_markers > num_current_markers:
        for i in range(num_current_markers, num_prev_markers):
            marker = Marker()
            marker.id = i
            marker.ns = marker_ns
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        num_prev_markers = num_current_markers
    
    cluster_pub.publish(marker_array)
    cluster_info_pub.publish(cluster_info_msg)

  

if __name__ == "__main__":
    rospy.init_node('mean_shift_clustering')

    ## Params
    quantile = rospy.get_param('~quantile', 0.5) #0.5
    frame = rospy.get_param('~frame', 'map') #'map'
    marker_ns = rospy.get_param('~uav_cluster', 'uav_cluster')

    rospy.Subscriber('obstacle_voxel', PoseArray, callback)
    cluster_pub = rospy.Publisher('obstacle_cluster_markers', MarkerArray, queue_size=10)
    cluster_info_pub = rospy.Publisher('obstacle_cluster_info', ClusterInfo, queue_size=10)
    rospy.spin()

    