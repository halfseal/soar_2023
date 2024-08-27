import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import GridCells
from px4_msgs.msg import VehicleOdometry
import matplotlib.pylab as plt

import struct
import math
import time


def read_points(msg):
    # Get field information
    fields = msg.fields
    field_names = [field.name for field in fields]
    field_offsets = [field.offset for field in fields]

    # Find the x, y, z fields
    x_idx = field_names.index("x")
    y_idx = field_names.index("y")
    z_idx = field_names.index("z")

    # Get data size and point step size
    data = msg.data
    data_size = len(data)
    point_step = msg.point_step

    # Extract x, y, z coordinates from the data using np.frombuffer()
    data_buffer = np.frombuffer(data, dtype=np.float32)

    # Reshape the data buffer to match the structure of the point cloud
    point_data = data_buffer.reshape(-1, point_step // 4)

    # Perform point validity checks and create points array
    mask = (
        np.isfinite(point_data[:, x_idx])
        & np.isfinite(point_data[:, y_idx])
        & np.isfinite(point_data[:, z_idx])
    )
    valid_points = point_data[mask]

    return np.hstack(
        (valid_points[:, :3], np.ones((valid_points.shape[0], 1), dtype=np.float32))
    )


def quaternion_to_rotmat(o4x1: np.array) -> np.array:
    # quaternion to rotation matrix
    o4x1 /= np.linalg.norm(o4x1)
    w, x, y, z = o4x1

    rotation_matrix = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
        ]
    )

    return rotation_matrix

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__("pointcloud_subscriber")

        self.get_logger().info("map_talker initiated!")

        self.point_subscription = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/mapping/fused_cloud",
            self.pointcloud_callback,
            10,
        )
        self.point_subscription

        self.pose_subscription = self.create_subscription(
            VehicleOdometry, "/fmu/vehicle_odometry/out", self.pose_callback, 10
        )
        self.pose_subscription  # prevent unused variable warning

        self.marker_publisher = self.create_publisher(Marker, "/map_talker/marker", 10)
        self.grid_publisher = self.create_publisher(
            GridCells, "/map_talker/gridcells", 10
        )
        self.grid_publisher2 = self.create_publisher(
            GridCells, "/map_talker/gridcells2", 10
        )

        self.voxel_size = 0.2
        self.pcd = o3d.geometry.PointCloud()

        self.rotmat = None
        self.position = None

        # only for visualization
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, "/map_talker/pc2", 10
        )


    def pointcloud_callback(self, msg):
        if self.rotmat is None:
            return 
        t = time.time()

        pcmsg = PointCloud2()
        pcmsg.header.frame_id = msg.header.frame_id
        pcmsg.height = msg.height
        pcmsg.width = msg.width
        pcmsg.fields = msg.fields
        pcmsg.is_bigendian = msg.is_bigendian
        pcmsg.point_step = msg.point_step
        pcmsg.row_step = msg.row_step
        pcmsg.data = msg.data

        self.pointcloud_publisher.publish(pcmsg)

        points = read_points(msg)
        points = points.reshape(len(points), 4, 1)
        points = points[:, :3, :]
        points = points.reshape(len(points), 3, 1)

        #   N                 1  0  0     cx     tx
        #   E  =  quatmat  x  0 -1  0  x  cy  +  ty
        #   D                 0  0 -1     cz     tz
        cmat = np.array(
            [
                [1, 0, 0],
                [0, -1, 0],
                [0, 0, -1],
            ]
        )
        points = np.matmul(cmat, points)
        points = np.matmul(self.rotmat, points)
        

        points = points.reshape(len(points), 3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd = pcd
        # o3d.io.write_point_cloud("pcd_from_msg.pcd", self.pcd, True)

        # publishing marker is only for debugging purpose
        self.marker_publish(msg.header.frame_id)
        self.grid_publish(msg.header.frame_id)
        self.get_logger().info(f"total time cost : {time.time() - t}")


    def marker_publish(self, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.DELETEALL
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.marker_publisher.publish(marker)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        points = np.asarray(self.pcd.points)
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            marker.points.append(p)

        self.marker_publisher.publish(marker)

    def grid_publish(self, frame_id):
        if self.position is None:
            return

        # initializing GridCells msg
        grid_cells_msg = GridCells()
        grid_cells_msg.header.frame_id = frame_id
        grid_cells_msg.cell_width = self.voxel_size
        grid_cells_msg.cell_height = self.voxel_size

        grid_cells_msg2 = GridCells()
        grid_cells_msg2.header.frame_id = frame_id
        grid_cells_msg2.cell_width = self.voxel_size
        grid_cells_msg2.cell_height = self.voxel_size

        # threshold for x,y position ± 14.0
        x_threshold_lower = self.position[0] - 14.0
        x_threshold_upper = self.position[0] + 14.0

        y_threshold_lower = self.position[1] - 14.0
        y_threshold_upper = self.position[1] + 14.0

        # threshold for z position ± voxel_size(20.0 currently)
        z_threshold_lower = self.position[2] - self.voxel_size
        z_threshold_upper = self.position[2] + self.voxel_size

        # clipping by threshold
        points = np.asarray(self.pcd.points)
        mask = ((points[:, 2] > z_threshold_lower) & (points[:, 2] < z_threshold_upper)
            & (points[:, 0] > x_threshold_lower) & (points[:, 0] < x_threshold_upper)
            & (points[:, 1] > y_threshold_lower) & (points[:, 1] < y_threshold_upper))
        selected_points = points[mask]
        selected_points[:, 2] = self.position[2]

        # make pcd with masked points
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(selected_points)
        # pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        if len(pcd.points) <= 0 :
            return

        # labels = np.array(pcd.cluster_dbscan(eps=self.voxel_size*2, min_points=10, print_progress=True))
        labels = np.array(pcd.cluster_dbscan(eps=self.voxel_size*2, min_points=5))
        max_label = labels.max()
        self.get_logger().info(f"point cloud has {max_label + 1} clusters")
        
        # # coloring. only for debuging purpose
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.io.write_point_cloud("pcd_from_msg_cluster.pcd", pcd, True)

        # deviding clusters by label
        valid_labels = labels != -1
        splited_points = np.hstack((selected_points[valid_labels], labels[valid_labels, None]))
        splited_points = splited_points[splited_points[:, -1].argsort()]
        splited_points = np.split(splited_points[:,:-1], np.unique(splited_points[:, -1], return_index=True)[1][1:])
        self.get_logger().info(f"num of clusters: {len(splited_points)}")

        for i in range(0, len(splited_points)):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(splited_points[i])
            center = pcd.get_center()
            p = Point()
            p.x = center[0]
            p.y = center[1]
            p.z = center[2]
            grid_cells_msg.cells.append(p)

        for point in selected_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            grid_cells_msg2.cells.append(p)
        self.get_logger().info(f"grid cell num: {len(selected_points)}")
        self.grid_publisher.publish(grid_cells_msg)
        self.grid_publisher2.publish(grid_cells_msg2)

    def pose_callback(self, msg):
        self.position = np.array([msg.position[0], msg.position[1], msg.position[2]])
        
        if self.rotmat is not None:
            return
        self.get_logger().info(f"rotation matrix for NED set!")
        orientation = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        self.rotmat = quaternion_to_rotmat(orientation)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()