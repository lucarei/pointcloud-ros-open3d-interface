import rospy
import open3d_conversions
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import math
from math import pi
import itertools
from std_msgs.msg import Int32,String

rospy.init_node('open3d_conversions_example')

current_cloud = None

def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(10)

listener = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, handle_pointcloud, queue_size=1)
publisher = rospy.Publisher('~processed_point_cloud', PointCloud2, queue_size=1)
pub=rospy.Publisher("pose", String, queue_size=1)

while not rospy.is_shutdown():
    
    if current_cloud is None:
        continue

    print(current_cloud.header.frame_id)

    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    pcd=o3d_cloud

    # Create bounding box:
    bounds = [ [-0.3, 0.3], [0, 3], [-0.3, 1.37]]  # set the bounds
    bounding_box_points = list(itertools.product(*bounds))  # create limit points
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object

    # Crop the point cloud using the bounding box:
    pcd_croped = pcd.crop(bounding_box)

    # Display the cropped point cloud:
    #o3d.visualization.draw_geometries([pcd_croped])

    obb = pcd_croped.get_minimal_oriented_bounding_box() #seems to be the best
    
    obb.color = (0, 1, 0)


    #print(obb)
    # o3d.visualization.draw_geometries([chair,obb],
    #                               zoom=0.7,
    #                               front=[0.5439, -0.2333, -0.8060],
    #                               lookat=[2.4615, 2.1331, 1.338],
    #                               up=[-0.1781, -0.9708, 0.1608])

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
    info=str(obb.center[0])+","+str(obb.center[1])+","+str(obb.center[2])+","+str(angles[0])+","+str(angles[1])+","+str(angles[2])+","+str(angles[3])
    pub.publish(info)
    o3d_cloud=pcd_croped
    
    ros_cloud = open3d_conversions.to_msg(o3d_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    publisher.publish(ros_cloud)
        
    current_cloud = None
    rate.sleep()

