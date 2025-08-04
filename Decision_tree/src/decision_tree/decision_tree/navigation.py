import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
from enum import Enum
from .utils import ns_topic
from .verify_connection import ConnectionStatus

class Navigation:
    def __init__(self, node):
        self.node = node
        self.nav_action_client = ActionClient(
            node, NavigateToPose, 
            ns_topic(node, 'navigate_to_pose')
        )
        self.position_tolerance = node.get_parameter('position_tolerance').value
        self.robot_base_frame = node.get_parameter('robot_base_frame').value
        self.current_goal_handle = None
        self.current_target = None

    def navigate_to_point(self, x, y):
        """导航到指定点"""
        if self.node.connection_manager.connection_status != ConnectionStatus.CONNECTED:
            self.node.get_logger().error("未连接到机器人，无法导航")
            return False
        if self.node.is_dead:
            self.node.get_logger().warn("机器人已死亡，无法导航")
            return False
        if not self.node.connection_manager.validate_connection():
            self.node.get_logger().error("连接验证失败，无法导航")
            return False
            
        # 检查当前位置与目标位置的距离
        position_yaw = self.get_robot_position()
        if position_yaw[0]:
            (cx, cy), _ = position_yaw
            distance = math.sqrt((cx - x)**2 + (cy - y)** 2)
            if distance < self.position_tolerance:
                self.node.get_logger().info(f"已在目标点附近 (距离: {distance:.2f}m), 跳过导航")
                self.node.is_navigating = False
                self.current_target = None
                self.node.initial_navigation_complete = True
                return
        
        self.cancel_navigation()
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.nav_action_client.wait_for_server()
        self.send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.current_target = (x, y)
        self.node.is_navigating = True
        self.node.get_logger().info(f"导航到目标点: ({x:.2f}, {y:.2f})")
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('目标被拒绝')
            self.node.is_navigating = False
            self.current_goal_handle = None
            return
        
        self.node.get_logger().info('目标接受')
        self.current_goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'剩余距离: {feedback.distance_remaining:.2f}米', throttle_duration_sec=2)

    def nav_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'导航完成，结果: {result.result}')
        self.node.is_navigating = False
        self.current_goal_handle = None
        self.node.initial_navigation_complete = True

    def cancel_navigation(self):
        """取消当前导航任务"""
        if self.current_goal_handle is not None:
            self.node.get_logger().info("取消当前导航")
            # 异步取消
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)
        self.node.is_navigating = False
        self.current_target = None
        self.current_goal_handle = None

    def cancel_done(self, future):
        """导航取消完成回调"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info("导航成功取消")
        else:
            self.node.get_logger().warn("取消导航失败")

    def get_robot_position(self):
        """获取机器人当前位置和朝向"""
        try:
            if not self.node.connection_manager.tf_buffer.can_transform(
                'map', self.robot_base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)):
                self.node.get_logger().warn(f"无法获取位置: 无'map'到'{self.robot_base_frame}'的变换", throttle_duration_sec=5)
                return None, None
                
            transform = self.node.connection_manager.tf_buffer.lookup_transform(
                'map', self.robot_base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            position = transform.transform.translation
            orientation = transform.transform.rotation
            
            # 将四元数转换为偏航角
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (position.x, position.y), yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().warn(f"获取位置失败: {str(e)}", throttle_duration_sec=5)
            return None, None
        except Exception as e:
            self.node.get_logger().error(f"获取位置异常: {str(e)}")
            return None, None