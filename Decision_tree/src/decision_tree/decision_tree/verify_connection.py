from enum import Enum
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class ConnectionStatus(Enum):
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    ERROR = 3

class VerifyConnection:
    def __init__(self, node):
        self.node = node
        self.connection_status = ConnectionStatus.DISCONNECTED
        self.connection_retry_count = 0
        self.connection_timer = None
        self.max_retry_count = node.get_parameter('max_connection_retries').value
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        
        # 动作客户端
        self.nav_action_client = ActionClient(
            node, NavigateToPose, 
            self._ns_topic('navigate_to_pose')
        )

    def start_connection_check(self):
        """启动连接检查"""
        self.node.get_logger().info("开始系统自检...")
        self.connection_status = ConnectionStatus.CONNECTING
        self.connection_retry_count = 0
        self.check_connection()

    def check_connection(self):
        """检查与机器人的连接状态"""
        if self.connection_status == ConnectionStatus.CONNECTED:
            return
            
        self.node.get_logger().info(f"执行连接检查 (尝试 #{self.connection_retry_count+1})")
        
        # 检查TF连接
        tf_ok = self.check_tf_connection()
        if not tf_ok:
            self.handle_connection_failure("TF连接失败")
            return
        
        # 检查导航服务器
        nav_ok = self.check_navigation_server()
        if not nav_ok:
            self.handle_connection_failure("导航服务器不可用")
            return
        
        # 所有检查通过
        self.connection_status = ConnectionStatus.CONNECTED
        self.node.get_logger().info("所有自检通过! 系统准备就绪")
        
        # 初始化导航
        self.node.initial_navigation_complete = True
        self.cancel_connection_timer()

    def handle_connection_failure(self, reason):
        """处理连接失败情况"""
        self.connection_retry_count += 1
        self.node.get_logger().error(f"{reason} (尝试 #{self.connection_retry_count})")
        
        if self.connection_retry_count >= self.max_retry_count:
            self.connection_status = ConnectionStatus.ERROR
            self.node.get_logger().error('检查namespace和TF连接是否正确，或检查机器人是否在线。')
            return
        
        if self.connection_timer:
            self.connection_timer.cancel()
        
        self.node.get_logger().info(f"5秒后重试连接...")
        self.connection_timer = self.node.create_timer(5.0, self.check_connection)
        
    def cancel_connection_timer(self):
        """取消连接检查定时器"""
        if self.connection_timer:
            self.connection_timer.cancel()
            self.connection_timer = None

    def validate_connection(self):
        """验证当前连接是否仍然有效"""
        try:
            if not self.tf_buffer.can_transform(
                'map', self.node.robot_base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=1.0)):
                self.node.get_logger().warn("连接验证失败: TF变换不可用")
                return False
                
            if not self.nav_action_client.server_is_ready():
                self.node.get_logger().warn("连接验证失败: 导航服务器无响应")
                return False
                
            return True
        except Exception as e:
            self.node.get_logger().warn(f"连接验证异常: {str(e)}")
            return False
        
    def check_tf_connection(self):
        """TF连接检查"""
        try:
            if not self.tf_buffer.can_transform('map', self.node.robot_base_frame, rclpy.time.Time()):
                self.node.get_logger().warn(f"缺少从'map'到'{self.node.robot_base_frame}'的变换")
                return False
            
            # 获取变换以验证其有效性
            self.tf_buffer.lookup_transform(
                'map', self.node.robot_base_frame, rclpy.time.Time())
            
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().warn(f"TF错误: {str(e)}")
            return False
        except Exception as e:
            self.node.get_logger().error(f"TF连接检查异常: {str(e)}")
            return False
    
    def check_navigation_server(self):
        """检查导航服务器是否可用"""
        try:
            if not self.nav_action_client.wait_for_server(timeout_sec=3.0):
                self.node.get_logger().warn("导航服务器未响应")
                return False
            return True
        except Exception as e:
            self.node.get_logger().error(f"导航服务器检查异常: {str(e)}")
            return False

    def _ns_topic(self, topic):
        """根据命名空间拼接话题或服务名"""
        node_ns = self.node.get_namespace()
        
        if node_ns.startswith('/'):
            node_ns = node_ns[1:]
        
        if topic.startswith('/'):
            topic = topic[1:]
            
        if node_ns:
            return f"/{node_ns}/{topic}"
        else:
            return f"/{topic}"