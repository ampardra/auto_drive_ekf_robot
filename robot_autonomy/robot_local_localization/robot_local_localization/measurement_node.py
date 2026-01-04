#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MeasurementNode(Node):
    def __init__(self):
        super().__init__("measurement_node")
        
        # 1. Parameters
        self.declare_parameter("alpha", 0.5)
        self.declare_parameter("imu_topic", "/zed/zed_node/imu/data_raw")
        self.declare_parameter("vo_topic", "/vo/odom")
        
        self.alpha = self.get_parameter("alpha").value
        imu_topic_name = self.get_parameter("imu_topic").value
        vo_topic_name = self.get_parameter("vo_topic").value

        # 2. QoS Policy: LISTEN TO EVERYTHING (Best Effort & Reliable)
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 3. Subscribers
        self.imu_sub = self.create_subscription(Imu, imu_topic_name, self.imu_cb, qos_policy)
        self.vo_sub = self.create_subscription(Odometry, vo_topic_name, self.vo_cb, qos_policy)
        
        # 4. Publisher
        self.pub = self.create_publisher(Odometry, "/measurement_model", 10)
        
        # 5. Storage
        self.last_imu = None
        self.last_vo = None
        
        self.get_logger().info(f"Measurement Node Started.")
        self.get_logger().info(f"Waiting for IMU on: {imu_topic_name}")
        self.get_logger().info(f"Waiting for VO on: {vo_topic_name}")

    def imu_cb(self, msg):
        # Debug Log: Print once just to prove connection
        if self.last_imu is None:
            self.get_logger().info("First IMU message received!")
        self.last_imu = msg
        self.compute_and_publish()

    def vo_cb(self, msg):
        # Debug Log: Print once just to prove connection
        if self.last_vo is None:
            self.get_logger().info("First VO message received!")
        self.last_vo = msg
        self.compute_and_publish()

    def compute_and_publish(self):
        if self.last_imu is None or self.last_vo is None:
            return

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link_measurement" # Unique frame!
        
  
        out.pose.pose.position.x = self.last_vo.pose.pose.position.x
        out.pose.pose.position.y = self.last_vo.pose.pose.position.y
        out.pose.pose.position.z = 0.0 
        
        # Use IMU for orientation
        out.pose.pose.orientation = self.last_imu.orientation
        
        self.pub.publish(out)
        
        # Debug: Uncomment to see the stream
        # self.get_logger().info(f"Published Measurement: x={out.pose.pose.position.x:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()