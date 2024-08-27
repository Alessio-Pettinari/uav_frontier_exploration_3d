#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import MeanShift, estimate_bandwidth, KMeans, DBSCAN
from uav_frontier_exploration_3d.msg import ClusterInfo, FreeMeshInfo
from scipy.spatial import ConvexHull, Delaunay, KDTree
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


freeSpace_reached = False
frontier_reached = False
num_prev_markers = 0

# octomap_res = 0.1*2

# def find_nearest_within_threshold(point, kdtree, points, octomap_res):
#     distances, indices = kdtree.query(point, k=2)  # k=2 include il punto stesso e il più vicino
#     if distances[1] <= octomap_res:
#         return points[indices[1]]
#     else:
#         return None

# def create_parallelepiped_marker(id, min_point, max_point):
#     marker = Marker()
#     marker.header.frame_id = frame  # Modifica se necessario
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = marker_ns
#     marker.id = id
#     marker.type = Marker.CUBE
#     marker.action = Marker.ADD

#     # Calcolo del centro del parallelepipedo
#     center = [(min_point[0] + max_point[0]) / 2.0,
#               (min_point[1] + max_point[1]) / 2.0,
#               (min_point[2] + max_point[2]) / 2.0]
    
#     marker.pose.position.x = center[0]
#     marker.pose.position.y = center[1]
#     marker.pose.position.z = center[2]

#     # Dimensioni del parallelepipedo
#     marker.scale.x = max_point[0] - min_point[0]
#     marker.scale.y = max_point[1] - min_point[1]
#     marker.scale.z = max_point[2] - min_point[2]

#     # Colore del parallelepipedo
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.color.a = 0.5  # Trasparenza

#     return marker

# def publish_markers(parallelepipeds):
#     marker_array = MarkerArray()
#     for i, (min_point, max_point) in enumerate(parallelepipeds):
#         marker = create_parallelepiped_marker(i, min_point, max_point)
#         marker_array.markers.append(marker)

#     cluster_pub.publish(marker_array)

# def callback(pose_array):
#     points = []
#     for pose in pose_array.poses:
#         points.append([pose.position.x, pose.position.y, pose.position.z]) 

#     points = np.array(points)

#     if len(points) == 0:
#         rospy.loginfo("Cluster points array empty!")
#         return

#     kdtree = KDTree(points)
#     parallelepipeds = []
#     visited_points = set()

#     # Clusterizzazione e costruzione dei parallelepipedi
#     for point in points:
#         if tuple(point) in visited_points:
#             continue
        
#         cluster_points = []
#         to_visit = [point]
        
#         while to_visit:
#             current_point = to_visit.pop()
#             if tuple(current_point) in visited_points:
#                 continue
            
#             visited_points.add(tuple(current_point))
#             cluster_points.append(current_point)
            
#             nearest = find_nearest_within_threshold(current_point, kdtree, points, octomap_res)
#             if nearest is not None and tuple(nearest) not in visited_points:
#                 to_visit.append(nearest)
        
#         # Se il cluster è stato creato, costruisci il parallelepipedo
#         if cluster_points:
#             cluster_points = np.array(cluster_points)
#             min_x, min_y, min_z = np.min(cluster_points, axis=0)
#             max_x, max_y, max_z = np.max(cluster_points, axis=0)
            
#             parallelepipeds.append(((min_x, min_y, min_z), (max_x, max_y, max_z)))

#     # Pubblica i marker
#     publish_markers(parallelepipeds)

##############################################################

## Callback Clustering Obstacle:
def callback(pose_array):
    global num_prev_markers, obstacle_reached
    # Estraggo i punti dal messaggio PoseArray:
    # il topic geometry_msgs::PoseArray contiene un array di pose, dove c'è la proprietà "poses" che è un messaggio di tipo geometry_msgs::Pose
    points = []
    for pose in pose_array.poses:
        points.append([pose.position.x, pose.position.y, pose.position.z]) 

    # Converto in un array numPy
    points = np.array(points)

    if len(points) == 0:
        rospy.loginfo("Cluster points array empty!")
        
    # Clustering MEAN-SHIFT
    bandwidth = estimate_bandwidth(points, quantile = quantile)
    if bandwidth <=0:
        bandwidth = 0.1  # Set a default value 
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(points)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_

    labels_unique = np.unique(labels)
    n_clusters_ = len(labels_unique)
    rospy.loginfo(f"Number of obstacle clusters : {n_clusters_}")

    ## K-MEANS
    # n_clusters = 200  # Imposta il numero desiderato di cluster
    # kmeans = KMeans(n_clusters=n_clusters)
    # kmeans.fit(points)
    # labels = kmeans.labels_
    # cluster_centers = kmeans.cluster_centers_

    # labels_unique = np.unique(labels)
    # n_clusters_ = len(labels_unique)
    # rospy.loginfo(f"number of obstacle clusters : {n_clusters_}")


    ## Initializing msg custom 
    cluster_info_msg = ClusterInfo()
    cluster_info_msg.clst_centers = [] # attributi msg custom
    cluster_info_msg.clst_radius = []  # attributi msg custom

    # Marker dei cluster  per Rviz
    # marker_array = MarkerArray()
    # for i, center in enumerate(cluster_centers):
    #     # Centers and radius of clustered spheres  
    #     cluster_points = points[labels == i]
    #     radius_cluster = np.max(np.linalg.norm(cluster_points - center, axis=1))  # max distanza tra centro del cluster e i punti, che viene utilizzato come raggio della sfera.
                                                                                # (np.linalg.norm calcola la norma di ogni vettore spostamento. axis=1 indica che la norma deve essere calcolata lungo l'asse colonne)

    marker_array = MarkerArray()

    for i, center in enumerate(cluster_centers):
        # Centers and radius of clustered spheres  
        cluster_points = points[labels == i]
        if len(cluster_points) == 0:
            rospy.logwarn(f"Cluster {i} is empty. This one is skipped!")
            continue   # skip this cluster and move on to the next one
        radius_cluster = np.max(np.linalg.norm(cluster_points - center, axis=1))

        # Check cluster radius dimension: if it is greater than the threshold, the cluster is re-performed 
        if radius_cluster > radius_threshold:
        #     bandwidth_sub = estimate_bandwidth(cluster_points, quantile = quantile)
        #     if bandwidth_sub <= 0:
        #         bandwidth_sub = 0.1
        #     ms_sub = MeanShift(bandwidth=bandwidth_sub, bin_seeding=True)
            ms_sub=KMeans(n_clusters=3, n_init=10)
            ms_sub.fit(cluster_points)
            sub_labels = ms_sub.labels_
            sub_centers = ms_sub.cluster_centers_

            for j, sub_center in enumerate(sub_centers):
                sub_cluster_points = cluster_points[sub_labels == j]
                if len(sub_cluster_points) == 0:
                    rospy.logwarn(f"Sub-Cluster {j} is empty. This one is skipped!")
                    continue   # skip this cluster and move on to the next one

                sub_radius_cluster = np.max(np.linalg.norm(sub_cluster_points - sub_center, axis=1))

                # Marker Sub-cluster:
                marker = Marker()
                marker.id = i
                marker.ns = marker_ns
                marker.header.frame_id = frame
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = sub_center[0]
                marker.pose.position.y = sub_center[1]
                marker.pose.position.z = sub_center[2]
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = sub_radius_cluster*2 # scale è riferito al diametro!
                marker.scale.y = sub_radius_cluster*2
                marker.scale.z = sub_radius_cluster*2
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 139
                marker.color.a = 0.3
                marker_array.markers.append(marker)

                center_point = Point(x=sub_center[0], y=sub_center[1], z=sub_center[2])
                cluster_info_msg.clst_centers.append(center_point)
                cluster_info_msg.clst_radius.append(sub_radius_cluster)
        else:
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

            center_point = Point(x=center[0], y=center[1], z=center[2])                
            cluster_info_msg.clst_centers.append(center_point)
            cluster_info_msg.clst_radius.append(radius_cluster)


        # print(f"Cluster {i}: Center = {center}, Radius = {radius_cluster}")
        # marker = Marker()
        # marker.id = i
        # marker.ns = marker_ns
        # marker.header.frame_id = frame
        # marker.type = Marker.SPHERE
        # marker.action = Marker.ADD
        # marker.pose.position.x = center[0]
        # marker.pose.position.y = center[1]
        # marker.pose.position.z = center[2]
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0
        # marker.scale.x = radius_cluster*2 # scale è riferito al diametro!
        # marker.scale.y = radius_cluster*2
        # marker.scale.z = radius_cluster*2
        # marker.color.r = 0
        # marker.color.g = 255
        # marker.color.b = 255
        # marker.color.a = 0.3
        # marker_array.markers.append(marker)

        # # Creo il topic custom msg (Point centers, float radius)
        # point = Point()
        # point.x = center[0]
        # point.y = center[1]
        # point.z = center[2]
        # cluster_info_msg.clst_centers.append(point)
        # cluster_info_msg.clst_radius.append(radius_cluster)


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
#####################################################################################

## Callback UAV Frontier:
def frontierCallback(pos):
    global frontier_pos, frontier_reached
    frontier_pos = []
    for marker in pos.markers:
            frontier_pos.append([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
    
    frontier_pos = np.array(frontier_pos)
    frontier_reached = True



## Callback free space:
def freeCallback(data):
    global freeSpace_reached, freeVoxel_points, free_cluster_pub
    freeVoxel_points = []

    for pose in data.poses:
        freeVoxel_points.append([pose.position.x, pose.position.y, pose.position.z])
    
    # Converto in un array numPy
    freeVoxel_points = np.array(freeVoxel_points)
    freeSpace_reached = True


def polyhedreFreeSpace():
    global freeVoxel_points, frontier_pos

    free_points = []
    # Add frontiers to free space
    free_points = np.vstack([freeVoxel_points, frontier_pos])
   
   
    ## DBSCAN
    # db = DBSCAN(eps=1, min_samples = 10).fit(free_points)
    # labels = db.labels_
    # # Number of clusters in labels, ignoring noise if present 
    # n_clusters_free = len(set(labels)) - (1 if -1 in labels else 0)
    # rospy.loginfo(f'Number of FreeVoxel clusters: {n_clusters_free}')

    # free_clusters = {i: [] for i in range(n_clusters_free)}
    # for pt, label in zip(free_points, labels):
    #     if label != -1:  # Delete noise
    #         free_clusters[label].append(pt)

####################################################
    # free_marker_array = MarkerArray()
    # for i, (label, points) in enumerate(free_clusters.items()):
    #     if len(points) < 3:
    #         continue

    #     points = np.array(points)
    #     hull = ConvexHull(points)

    #     marker = Marker()
    #     marker.header.frame_id = frame
    #     marker.header.stamp = rospy.Time.now()
    #     marker.ns = "clusters"
    #     marker.id = i
    #     marker.type = Marker.LINE_STRIP
    #     marker.action = Marker.ADD
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0
    #     marker.scale.x = 0.1

    #     for vertex in hull.vertices:
    #         pt = points[vertex]
    #         p = Point()
    #         p.x, p.y, p.z = pt
    #         marker.points.append(p)
    #     # Chiudi il poligono
    #     marker.points.append(marker.points[0])     
    #     free_marker_array.markers.append(marker)
###############################################


##  CONVEX HULL

    equation_mesh = FreeMeshInfo()
    equation_mesh.coeff = []
    equation_mesh.const = []


    hull = ConvexHull(free_points)
    hull_faces = hull.simplices

    # for cluster_id, points in free_clusters.items():
    if len(free_points) < 3:
        # continue
        return

    # points_hull = np.array(points)


    free_marker_array = MarkerArray()
    marker_id = 0

    marker1 = Marker()
    marker1.header.frame_id = frame
    marker1.header.stamp = rospy.Time.now()
    marker1.ns = "convex_hull"
    marker1.id = marker_id
    marker_id += 1
    marker1.type = Marker.TRIANGLE_LIST
    marker1.action = Marker.ADD
    marker1.pose.orientation.w = 1.0
    marker1.scale.x = 1.0
    marker1.scale.y = 1.0
    marker1.scale.z = 1.0
    marker1.color.r = 0.0
    marker1.color.g = 1.0
    marker1.color.b = 0.0
    marker1.color.a = 0.5

    # Add mesh points
    for simplex in hull_faces:
        for i in simplex:
            p = Point()
            # p.x, p.y, p.z = points_hull[i]
            p.x, p.y, p.z = free_points[i]
            marker1.points.append(p)

    free_marker_array.markers.append(marker1)


    # Create equation faces (a*x <= -d)
    for eq in hull.equations:
        eq_coeff = eq[:-1] #estrae tutti gli elementi dell'eq dell'iperpiano tranne l'ultimo
        eq_offset = eq[-1] 
        # equation_mesh.coeff = list(eq_coeff)
        # equation_mesh.const = eq_offset
        equation_mesh.coeff.extend(eq_coeff)
        equation_mesh.const.append(eq_offset)
    
    free_cluster_pub.publish(free_marker_array)
    eq_convexHull_pub.publish(equation_mesh)



    ## ALPHA SHAPES
    # delaunay = Delaunay(free_points)
    # triangles = delaunay.simplices
    # rospy.loginfo(f"Triangles: {triangles}")
    # # edges = np.vstack([triangles[:,[0, 1]], triangles[:,[1,2]], triangles[:,[2,0]]])
    # # edges = np.unique(np.sort(edges, axis=1), axis=0)
    # # edge_lengths = cdist(free_points[edges[:, 0]], free_points[edges[:, 1]])
    # # is_alpha = edge_lengths.max(axis=1) < alpha

    # alpha_shape = []
    # alpha = 0.2
    # # for i, tri in enumerate(triangles):
    # #     pts = free_points[tri]
    # #     distances = cdist(pts, pts)
    # #     if np.all(distances < alpha):
    # #         alpha_shape.append(pts)
    # for tri in triangles:
    #     # Assicurati che 'tri' sia un array numpy e abbia la forma corretta
    #     if isinstance(tri, np.ndarray) and tri.ndim == 1 and len(tri) == 4:
    #         pts = free_points[tri]  # Usa gli indici per ottenere i punti
    #         distances = cdist(pts, pts)
    #         rospy.loginfo(f"Distances for triangle {tri}: {distances}")
    #         if np.all(distances < alpha):
    #             alpha_shape.append(pts)
    #             rospy.loginfo(f"Valid alpha shape triangle: {pts}")
    #         else:
    #             rospy.logwarn(f"Triangle {tri} does not satisfy alpha condition.")
    #     else:
    #         rospy.logwarn(f"Triangle has incorrect shape: {tri.shape}")

    # if not alpha_shape:
    #     rospy.logwarn("Alpha shape is empty, no valid points found.")
    #     return


    # marker_array = MarkerArray()
    # for i, shape in enumerate(alpha_shape):
    #     marker = Marker()
    #     marker.header.frame_id = "odom"
    #     marker.header.stamp = rospy.Time.now()
    #     marker.ns = "alpha_shape"
    #     marker.id = i
    #     marker.type = Marker.TRIANGLE_LIST
    #     marker.action = Marker.ADD
    #     marker.scale.x = 1.0
    #     marker.scale.y = 1.0
    #     marker.scale.z = 1.0
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 0.5
    #     marker.pose.orientation.w = 1.0  

    #     rospy.loginfo(f"Creating marker {i}")
    #     for tri in shape:
    #         # Assicurati che 'tri' sia un array numpy e che abbia la forma corretta
    #         if isinstance(tri, np.ndarray) and tri.ndim == 2 and tri.shape[1] == 3:
    #             for point in tri:
    #                 if isinstance(point, np.ndarray) and point.size == 3:
    #                     p = Point()
    #                     try:
    #                         p.x, p.y, p.z = point.tolist()
    #                         marker.points.append(p)
    #                         rospy.loginfo(f"Added point: {p}")
    #                     except ValueError as e:
    #                         rospy.logwarn(f"ValueError: {e} for point: {point}")
    #                 else:
    #                     rospy.logwarn(f"Invalid point format: {point}")
    #         else:
    #             rospy.logwarn(f"Triangle has incorrect shape: {tri.shape}")

    #     if not marker.points:
    #         rospy.logwarn(f"Marker {i} has no points!")
    #     else:
    #         marker_array.markers.append(marker)
    #         rospy.loginfo(f"Marker {i} added with {len(marker.points)} points")


    # free_cluster_pub.publish(marker_array)
    # rospy.loginfo("Markers published")



if __name__ == "__main__":
    rospy.init_node('mean_shift_clustering')

    ## Params
    quantile = rospy.get_param('~quantile', 0.5) 
    radius_threshold = rospy.get_param('~radius_threshold', 0.5) 
    frame = rospy.get_param('~frame', 'odom') 
    marker_ns = rospy.get_param('~uav_cluster', 'uav_cluster')

    rospy.Subscriber('obstacle_voxel', PoseArray, callback)
    rospy.Subscriber('free_voxel', PoseArray, freeCallback)
    rospy.Subscriber('uav_frontier', MarkerArray, frontierCallback)
    cluster_pub = rospy.Publisher('obstacle_cluster_markers', MarkerArray, queue_size=10)
    cluster_info_pub = rospy.Publisher('obstacle_cluster_info', ClusterInfo, queue_size=10)
    free_cluster_pub = rospy.Publisher('free_cluster_space', MarkerArray, queue_size=10)
    eq_convexHull_pub = rospy.Publisher('eq_convexHull', FreeMeshInfo, queue_size=10)


    rate = rospy.Rate(10)

    while not frontier_reached:
        rate.sleep()

    while not freeSpace_reached:
        rate.sleep()

    while not rospy.is_shutdown():
        if frontier_reached and freeSpace_reached:
            polyhedreFreeSpace()
        
        rate.sleep()


    rospy.spin()

    