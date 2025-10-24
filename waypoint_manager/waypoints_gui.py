#!/usr/bin/env python3
"""GUI that manages waypoints: subscribes to RViz topics and publishes nav_msgs/Path.

Features:
- Editable output topic textbox (QLineEdit) at the top of the GUI.
- `WaypointsGui.set_output_topic(topic)` method to recreate the publisher at runtime.
- Displays x, y, z, yaw (degrees) for each PoseStamped waypoint.
"""
import sys
import math
from copy import deepcopy

from PyQt5 import QtWidgets, QtCore

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointsGui(Node):
    def __init__(self):
        super().__init__('waypoints_gui')
        # parameters
        self.declare_parameter('pose_topic', '/goal_pose')
        self.declare_parameter('output_topic', 'waypoints')
        self.declare_parameter('default_frame', 'map')

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.default_frame = self.get_parameter('default_frame').get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        self._waypoints = []  # list of PoseStamped
        self._last_marker_count = 0

        # subscription
        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_cb, qos)

        # publisher (can be recreated by set_output_topic)
        self.pub = self.create_publisher(Path, self.output_topic, 10)
        # marker publisher for waypoint visualization
        self.marker_pub = self.create_publisher(MarkerArray, '~/waypoints_markers', 10)
        self.create_timer(1.0, self.publish_markers)

        # services
        self.clear_srv = self.create_service(Empty, 'clear_waypoints', self.clear_cb)
        self.publish_srv = self.create_service(Empty, 'publish_waypoints', self.publish_cb)

        self.get_logger().info(f'Waypoints GUI listening on {pose_topic}, publishing {self.output_topic}')

    def set_output_topic(self, topic: str):
        """Change the output topic at runtime by recreating the publisher.

        Safe for simple usage: destroys the old publisher reference and creates a new one.
        """
        if not topic:
            raise ValueError('topic must be non-empty')
        if topic == self.output_topic:
            return
        old = self.output_topic
        self.output_topic = topic
        # recreate publisher
        self.pub = self.create_publisher(Path, self.output_topic, 10)
        # recreate marker publisher to follow output topic (topic + '_markers')
        self.marker_pub = self.create_publisher(MarkerArray, f'{self.output_topic}_markers', 10)
        self.get_logger().info(f'Output topic changed from {old} to {self.output_topic}')

    def pose_cb(self, msg: PoseStamped):
        ps = deepcopy(msg)
        if not ps.header.frame_id:
            ps.header.frame_id = self.default_frame
        self._waypoints.append(ps)
        yaw = quaternion_to_yaw(ps.pose.orientation)
        self.get_logger().info(
            f'Added PoseStamped waypoint x={ps.pose.position.x:.3f} '
            f'y={ps.pose.position.y:.3f} z={ps.pose.position.z:.3f} '
            f'yaw={math.degrees(yaw):.1f}° frame={ps.header.frame_id}'
        )

    def publish_path(self):
        path = Path()
        if len(self._waypoints) == 0:
            return
        else:
            path.header = deepcopy(self._waypoints[0].header)
            path.header.stamp = self.get_clock().now().to_msg()
        path.poses = deepcopy(self._waypoints)
        self.pub.publish(path)
        # also publish arrow markers for visualization
        try:
            self.publish_markers()
        except Exception:
            # don't crash the node if markers fail
            self.get_logger().debug('Failed to publish markers')

    def clear_cb(self, request, response):
        self._waypoints = []
        self.get_logger().info('Cleared waypoints')
        self.publish_path()
        # ensure markers are cleared
        try:
            self.publish_markers()
        except Exception:
            pass
        return response

    def publish_cb(self, request, response):
        self.get_logger().info('Publish requested')
        self.publish_path()
        return response

    def get_poses(self):
        return self._waypoints

    def publish_markers(self):
        """Publish the current waypoints as visualization_msgs/MarkerArray of ARROW markers.

        Each waypoint becomes one Marker with semi-transparent alpha=0.5. Markers are published
        under the namespace 'waypoints' and ids are the waypoint indices. Markers that are no
        longer present are deleted by publishing DELETE actions for the leftover ids.
        """
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        # create an arrow marker for each waypoint
        for i, ps in enumerate(self._waypoints):
            m = Marker()
            m.header.frame_id = ps.header.frame_id if ps.header.frame_id else self.default_frame
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = deepcopy(ps.pose)
            # scale: x = length, y = shaft diameter, z = head diameter
            m.scale.x = 0.5
            m.scale.y = 0.06
            m.scale.z = 0.06
            # turquoise-ish color with 0.5 alpha
            m.color.r = 0.0
            m.color.g = 0.7
            m.color.b = 1.0
            m.color.a = 0.5
            markers.markers.append(m)

        # create a line strip marker that connects the waypoints (one marker)
        line = Marker()
        line.header.frame_id = self._waypoints[0].header.frame_id if len(self._waypoints) > 0 and self._waypoints[0].header.frame_id else self.default_frame
        line.header.stamp = now
        line.ns = 'waypoints_line'
        line.id = 0
        # LINE_STRIP connects points in order
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD if len(self._waypoints) > 0 else Marker.DELETE
        # line width
        line.scale.x = 0.04
        # semi-transparent white/green line
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.2
        line.color.a = 0.5
        # add points for the line
        if len(self._waypoints) > 0:
            for ps in self._waypoints:
                p = Point()
                p.x = ps.pose.position.x
                p.y = ps.pose.position.y
                p.z = ps.pose.position.z
                line.points.append(p)
            markers.markers.append(line)

        # remove any leftover markers from previous publish
        for i in range(len(self._waypoints), getattr(self, '_last_marker_count', 0)):
            md = Marker()
            md.header.frame_id = self.default_frame
            md.header.stamp = now
            md.ns = 'waypoints'
            md.id = i
            md.action = Marker.DELETE
            markers.markers.append(md)

        # publish and record count
        if len(markers.markers) > 0:
            self.marker_pub.publish(markers)
        self._last_marker_count = len(self._waypoints)


class MainWindow(QtWidgets.QWidget):
    def __init__(self, ros_node: WaypointsGui):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Waypoint Manager')
        self.resize(600, 420)

        # Widgets
        self.output_label = QtWidgets.QLabel('Output topic:')
        self.output_edit = QtWidgets.QLineEdit()
        self.output_edit.setText(self.ros_node.output_topic)
        self.output_edit.setFixedWidth(360)
        self.output_edit.setToolTip('Topic to publish the nav_msgs/Path to. Press Enter to apply.')

        self.list_widget = QtWidgets.QListWidget()

        self.remove_last_btn = QtWidgets.QPushButton('Remove')
        self.clear_btn = QtWidgets.QPushButton('Clear Waypoints')
        self.publish_btn = QtWidgets.QPushButton('Publish Path')

        # Layout
        layout = QtWidgets.QVBoxLayout()

        out_layout = QtWidgets.QHBoxLayout()
        out_layout.addWidget(self.output_label)
        out_layout.addWidget(self.output_edit)
        out_layout.addStretch()
        layout.addLayout(out_layout)

        layout.addWidget(self.list_widget)

        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.remove_last_btn)
        btn_layout.addWidget(self.clear_btn)
        btn_layout.addWidget(self.publish_btn)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

        # allow selecting individual waypoints in the list
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)

        # connections
        self.clear_btn.clicked.connect(self.on_clear)
        self.publish_btn.clicked.connect(self.on_publish)
        self.remove_last_btn.clicked.connect(self.on_remove)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_list)
        self.timer.start(200)  # update 5 Hz

        # wire the output topic edit to handler
        self.output_edit.editingFinished.connect(self.on_output_topic_changed)

    def on_output_topic_changed(self):
        new_topic = self.output_edit.text().strip()
        if not new_topic:
            QtWidgets.QMessageBox.warning(self, 'Waypoints', 'Topic cannot be empty')
            # restore current
            self.output_edit.setText(self.ros_node.output_topic)
            return
        if new_topic != self.ros_node.output_topic:
            try:
                self.ros_node.set_output_topic(new_topic)
                QtWidgets.QMessageBox.information(self, 'Waypoints', f'Output topic set to {new_topic}')
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, 'Waypoints', f'Failed to set output topic: {e}')
                # restore
                self.output_edit.setText(self.ros_node.output_topic)

    def update_list(self):
        # process ROS callbacks
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        poses = self.ros_node.get_poses()
        # remember current selection index so we can try to restore it after refresh
        sel = self.list_widget.currentRow()
        self.list_widget.clear()
        for i, ps in enumerate(poses):
            pos = ps.pose.position
            yaw = quaternion_to_yaw(ps.pose.orientation)
            yaw_deg = math.degrees(yaw)
            self.list_widget.addItem(f"{i}: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}, yaw={yaw_deg:.1f}° [{ps.header.frame_id}]")
        # restore selection if possible
        if sel >= 0 and sel < self.list_widget.count():
            self.list_widget.setCurrentRow(sel)

    def on_clear(self):
        # call service locally since the GUI owns the node
        self.ros_node.clear_cb(None, None)
        QtWidgets.QMessageBox.information(self, 'Waypoints', 'Cleared')

    def on_publish(self):
        self.ros_node.publish_cb(None, None)
        QtWidgets.QMessageBox.information(self, 'Waypoints', 'Published')

    def on_remove(self):
        poses = self.ros_node.get_poses()
        idx = self.list_widget.currentRow()
        if idx >= 0 and idx < len(poses):
            poses.pop(idx)
            self.ros_node.publish_path()
            QtWidgets.QMessageBox.information(self, 'Waypoints', 'Removed selected waypoint')
            # after removal, select the next valid row (same index)
            if idx < self.list_widget.count():
                self.list_widget.setCurrentRow(idx)
        else:
            QtWidgets.QMessageBox.information(self, 'Waypoints', 'No waypoint selected')


def main():
    rclpy.init()
    ros_node = WaypointsGui()
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(ros_node)
    w.show()
    try:
        app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
