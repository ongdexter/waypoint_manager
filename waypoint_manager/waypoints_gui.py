#!/usr/bin/env python3
"""GUI that manages waypoints: subscribes to RViz topics and publishes nav_msgs/Path."""
import sys
from copy import deepcopy

from PyQt6 import QtWidgets, QtCore

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty


def make_pose_from_point(ps_msg: PointStamped, default_z=0.0):
    p = PoseStamped()
    p.header = deepcopy(ps_msg.header)
    p.pose.position.x = ps_msg.point.x
    p.pose.position.y = ps_msg.point.y
    p.pose.position.z = ps_msg.point.z if hasattr(ps_msg, 'point') else default_z
    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = 0.0
    p.pose.orientation.w = 1.0
    return p


class WaypointsGui(Node):
    def __init__(self):
        super().__init__('waypoints_gui')
        # parameters
        self.declare_parameter('pose_topic', '/goal_pose')
        self.declare_parameter('point_topic', '/clicked_point')
        self.declare_parameter('output_topic', '/waypoints_path')
        self.declare_parameter('default_frame', 'map')

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        point_topic = self.get_parameter('point_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.default_frame = self.get_parameter('default_frame').get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        self._waypoints = []  # list of PoseStamped

        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self._pose_cb, qos)
        self.point_sub = self.create_subscription(PointStamped, point_topic, self._point_cb, qos)

        self.pub = self.create_publisher(Path, self.output_topic, 10)

        # services
        self.clear_srv = self.create_service(Empty, 'clear_waypoints', self._clear_cb)
        self.publish_srv = self.create_service(Empty, 'publish_waypoints', self._publish_cb)

        self.get_logger().info(f'Waypoints GUI listening on {pose_topic} and {point_topic}, publishing {self.output_topic}')

    def _pose_cb(self, msg: PoseStamped):
        ps = deepcopy(msg)
        if not ps.header.frame_id:
            ps.header.frame_id = self.default_frame
        self._waypoints.append(ps)
        self.get_logger().info(f'Added PoseStamped waypoint x={ps.pose.position.x:.3f} y={ps.pose.position.y:.3f} frame={ps.header.frame_id}')
        self._publish_path()

    def _point_cb(self, msg: PointStamped):
        ps = make_pose_from_point(msg)
        if not ps.header.frame_id:
            ps.header.frame_id = self.default_frame
        self._waypoints.append(ps)
        self.get_logger().info(f'Added PointStamped waypoint x={ps.pose.position.x:.3f} y={ps.pose.position.y:.3f} frame={ps.header.frame_id}')
        self._publish_path()

    def _publish_path(self):
        path = Path()
        if len(self._waypoints) == 0:
            path.header.frame_id = self.default_frame
            path.header.stamp = self.get_clock().now().to_msg()
        else:
            path.header = deepcopy(self._waypoints[0].header)
            path.header.stamp = self.get_clock().now().to_msg()
        path.poses = deepcopy(self._waypoints)
        self.pub.publish(path)

    def _clear_cb(self, request, response):
        self._waypoints = []
        self.get_logger().info('Cleared waypoints')
        self._publish_path()
        return response

    def _publish_cb(self, request, response):
        self.get_logger().info('Publish requested')
        self._publish_path()
        return response

    def get_poses(self):
        return self._waypoints


class MainWindow(QtWidgets.QWidget):
    def __init__(self, ros_node: WaypointsGui):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Waypoints')
        self.resize(500, 400)

        self.list_widget = QtWidgets.QListWidget()

        self.clear_btn = QtWidgets.QPushButton('Clear Waypoints')
        self.publish_btn = QtWidgets.QPushButton('Publish Path')
        self.remove_last_btn = QtWidgets.QPushButton('Remove Last')

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.list_widget)
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.clear_btn)
        btn_layout.addWidget(self.publish_btn)
        btn_layout.addWidget(self.remove_last_btn)
        layout.addLayout(btn_layout)
        self.setLayout(layout)

        # connections
        self.clear_btn.clicked.connect(self.on_clear)
        self.publish_btn.clicked.connect(self.on_publish)
        self.remove_last_btn.clicked.connect(self.on_remove_last)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_list)
        self.timer.start(200)  # update 5 Hz

    def update_list(self):
        # process ROS callbacks
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        poses = self.ros_node.get_poses()
        self.list_widget.clear()
        for i, ps in enumerate(poses):
            pos = ps.pose.position
            self.list_widget.addItem(f"{i}: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} frame={ps.header.frame_id}")

    def on_clear(self):
        # call service locally since the GUI owns the node
        self.ros_node._clear_cb(None, None)
        QtWidgets.QMessageBox.information(self, 'Waypoints', 'Cleared')

    def on_publish(self):
        self.ros_node._publish_cb(None, None)
        QtWidgets.QMessageBox.information(self, 'Waypoints', 'Published')

    def on_remove_last(self):
        poses = self.ros_node.get_poses()
        if poses:
            poses.pop()
            self.ros_node._publish_path()
            QtWidgets.QMessageBox.information(self, 'Waypoints', 'Removed last waypoint')
        else:
            QtWidgets.QMessageBox.information(self, 'Waypoints', 'No waypoints to remove')


def main():
    rclpy.init()
    ros_node = WaypointsGui()
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(ros_node)
    w.show()
    try:
        app.exec()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
