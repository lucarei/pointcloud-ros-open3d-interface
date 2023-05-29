import rospy
import open3d_conversions
from sensor_msgs.msg import PointCloud2

rospy.init_node('open3d_conversions_example')

current_cloud = None

def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(10)

listener = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, handle_pointcloud, queue_size=1)
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

    o3d_cloud=outlier_cloud
    
    ros_cloud = open3d_conversions.to_msg(o3d_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
    publisher.publish(ros_cloud)
        
    current_cloud = None
    rate.sleep()
