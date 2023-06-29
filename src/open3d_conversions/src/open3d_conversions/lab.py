import rospy
import open3d_conversions
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import math
from math import pi
import itertools

rospy.init_node('open3d_conversions_example')

current_cloud = None

def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(10)

listener = rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, handle_pointcloud, queue_size=1)
publisher = rospy.Publisher('~processed_point_cloud', PointCloud2, queue_size=1)

while not rospy.is_shutdown():
    
    if current_cloud is None:
        continue

    print(current_cloud.header.frame_id)

    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    pcd=o3d_cloud

    plane_model, inliers = o3d_cloud.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    
    pcd=outlier_cloud


    # # Create bounding box:bounds = [ [-0.5,0.5], [-math.inf,math.inf], [-0.12,0]]  # set the bounds
    # bounds = [ [-0.5,0.5], [-1,0], [-0.10,0]]  # set the bounds
    # bounding_box_points = list(itertools.product(*bounds))  # create limit points
    # bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
    #     o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object

    # # Crop the point cloud using the bounding box:
    # pcd = pcd.crop(bounding_box)
#####################ààààà
#SURFACE

    # alpha = 0.03
    # pcd=chair
    # print(f"alpha={alpha:.3f}")
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
    #     pcd, alpha)
    # mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)


##############################à
    chair=pcd


    #obb = chair.get_axis_aligned_bounding_box()
    #obb = chair.get_oriented_bounding_box() # works perfectly for yaw! --> bottle on floor 
    obb = chair.get_minimal_oriented_bounding_box() #seems to be the best
    
    obb.color = (0, 1, 0)


    #print(obb)
    o3d.visualization.draw_geometries([chair,obb],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])

###################################################
    rotation_matrix = obb.R

    ###########################################################
    
    from scipy.spatial.transform import Rotation 

    rotation_matrix=np.array([[rotation_matrix[0][0],rotation_matrix[0][1],rotation_matrix[0][2]],
                              [rotation_matrix[1][0],rotation_matrix[1][1],rotation_matrix[1][2]],
                              [rotation_matrix[2][0],rotation_matrix[2][1],rotation_matrix[2][2]]]) 
    #print(rotation_matrix)

    ### first transform the matrix to euler angles
    r =  Rotation.from_matrix(rotation_matrix)
    angles = r.as_quat()
    #print(angles)

    print(obb.center[0], obb.center[1], obb.center[2], angles[0],angles[1],angles[2],angles[3])

    # ###########################################
    # # Calcolare gli angoli di roll, pitch e yaw in radianti da rotation_matrix
    # yaw_rad = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    # pitch_rad = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    # roll_rad = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])


    # # Stampa dei risultati
    # print("Roll (radianti):", roll_rad)
    # print("Pitch (radianti):", pitch_rad)
    # print("Yaw (radianti):", yaw_rad)

    # #######################################à

    # axis_1 = obb.R[0]  # First axis
    # axis_2 = obb.R[1]  # Second axis
    # axis_3 = obb.R[2]  # Third axis

    # # Construct the rotation matrix
    # rotation_matrix = np.column_stack((axis_1, axis_2, axis_3))

    # # Print the rotation matrix
    # print("Rotation matrix:\n", rotation_matrix)
    # # Compute the RPY angles from the rotation matrix
    # roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    # pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    # yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    # # Convert the angles to degrees if desired
    # roll_deg = math.degrees(roll)
    # pitch_deg = math.degrees(pitch)
    # yaw_deg = math.degrees(yaw)

    # # Print the RPY angles
    # print("Roll (deg):", roll_deg)
    # print("Pitch (deg):", pitch_deg)
    # print("Yaw (deg):", yaw_deg)
    # print("Roll (deg):", roll_deg*pi/180)
    # print("Pitch (deg):", pitch_deg*pi/180)
    # print("Yaw (deg):", yaw_deg*pi/180)


    o3d_cloud=outlier_cloudewq
    
    ros_cloud = open3d_conversions.to_msg(o3d_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    publisher.publish(ros_cloud)
        
    current_cloud = None
    rate.sleep()
