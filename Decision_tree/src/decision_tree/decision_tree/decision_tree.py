import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import traceback
import numpy as np
import random
import math
import os
import pickle

from .verify_connection import VerifyConnection, ConnectionStatus
from .q_learning import Action, QLearning
from .navigation import Navigation
from .state_processor import StateProcessor
from .game_logic import GameLogic
from .exploration import Exploration
from .utils import ns_topic

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from robomaster_msgs.msg import RobotStatus  
from enemy_msgs.msg import EnemyState        
from nav_msgs.msg import OccupancyGrid       
from sensor_msgs.msg import JointState       

class ReinforcementLearningNavDecision(Node):
    def __init__(self):
        super().__init__('reinforcement_learning_nav_decision')
        
        # 最大重启次数
        self.max_restarts = 3
        self.restart_count = 0
        self.restart_requested = False
        
        self.map_data = None
        self.map_metadata = None
        
        try:
            self.initialize_node()
        except Exception as e:
            self.get_logger().error(f"节点初始化失败: {str(e)}")
            self.request_restart()

    def initialize_node(self):
        """节点初始化"""
        self.declare_parameters(
            namespace='',
            parameters=[
                # 基础位置参数
                ('base_position_x', 0.0),            # 基地在地图上的X坐标
                ('base_position_y', 0.0),            # 基地在地图上的Y坐标
                ('center_position_x', 4.5),          # 中心增益点X坐标
                ('center_position_y', -3.3),         # 中心增益点Y坐标
                ('center_radius', 0.5),              # 占领中心点的有效半径
                
                # 游戏机制参数
                ('base_heal_threshold', 50),         # 返回基地的生命值阈值
                ('victory_points_threshold', 200),   # 胜利所需的点数
                ('match_duration', 300),             # 比赛持续时间（秒）
                ('respawn_time_base', 10),           # 基础复活时间
                ('kill_points', 20),                 # 击杀敌方获得的点数
                ('death_points', 20),                # 死亡时敌方获得的点数
                ('center_point_rate', 1),            # 每秒占领中心点获得的点数
                
                # 强化学习参数
                ('learning_rate', 0.1),              # Q学习的学习率
                ('discount_factor', 0.95),           # Q学习的折扣因子
                ('exploration_rate', 0.3),           # 探索率（随机动作概率）
                ('q_table_path', 'Q/q_table.pkl'),   # Q表保存路径
                
                # 导航参数
                ('max_connection_retries', 5),       # 最大连接重试次数
                ('robot_base_frame', 'base_footprint'), # 机器人基础坐标系
                ('map_topic', '/map'),               # 地图话题
                ('decision_interval', 2.0),          # 决策间隔（秒）
                ('position_tolerance', 0.3),         # 位置容差（米）
                
                # 动作奖励参数
                ('reward_pursue', 1.0),              # 追击动作基础奖励
                ('reward_return_base', 0.5),         # 返回基地动作基础奖励
                ('reward_hold_position', 0.0),       # 保持位置动作基础奖励
                ('reward_capture_center', 2.0),      # 占领中心点动作基础奖励
                ('reward_explore_map', 1.5),         # 探索地图动作基础奖励
                ('reward_retreat', 0.8),             # 撤退动作基础奖励
                ('center_capture_boost', 3.0),       # 占领中心点的额外奖励
                ('explore_kill_bonus', 10.0),        # 探索中击杀的额外奖励
                ('retreat_success_bonus', 2.0),      # 成功撤退的额外奖励
                ('explore_new_point_reward', 0.1),   # 发现新区域的奖励
                
                # 探索参数
                ('explore_min_distance', 1.0),       # 探索点最小距离
                ('explore_priority_center', 0.6),    # 中心区域探索优先级
                ('explore_priority_base', 0.3),      # 基地区域探索优先级
                ('explore_priority_enemy', 0.1),     # 敌方区域探索优先级
                
                # 节点管理参数
                ('restart_delay', 5.0),              # 节点重启延迟(秒)
            ]
        )
        
        # 获取参数
        self.base_x = self.get_parameter('base_position_x').value
        self.base_y = self.get_parameter('base_position_y').value
        self.center_x = self.get_parameter('center_position_x').value
        self.center_y = self.get_parameter('center_position_y').value
        self.center_radius = self.get_parameter('center_radius').value
        self.heal_threshold = self.get_parameter('base_heal_threshold').value
        self.victory_threshold = self.get_parameter('victory_points_threshold').value
        self.match_duration = self.get_parameter('match_duration').value
        self.respawn_time_base = self.get_parameter('respawn_time_base').value
        self.kill_points = self.get_parameter('kill_points').value
        self.death_points = self.get_parameter('death_points').value
        self.center_point_rate = self.get_parameter('center_point_rate').value
        self.max_retry_count = self.get_parameter('max_connection_retries').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.map_topic = self.get_parameter('map_topic').value
        self.decision_interval = self.get_parameter('decision_interval').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.restart_delay = self.get_parameter('restart_delay').value
        
        # 计算敌方基地位置
        self.enemy_base_x = 2 * self.center_x - self.base_x
        self.enemy_base_y = 2 * self.center_y - self.base_y
        self.get_logger().info(f"计算敌方基地位置: ({self.enemy_base_x:.2f}, {self.enemy_base_y:.2f})")
        
        # 初始化模块
        self.connection_manager = VerifyConnection(self)
        self.q_learning = QLearning(self)
        self.navigation = Navigation(self)
        self.state_processor = StateProcessor(self)
        self.game_logic = GameLogic(self)
        self.exploration = Exploration(self)
        
        # 状态变量
        self.self_hp = 100
        self.self_vp = 0
        self.enemy_vp = 0
        self.is_active = False
        self.is_navigating = False
        self.current_target = None
        self.last_action = None
        self.last_state = None
        self.last_reward = 0
        self.current_goal_handle = None
        
        # 敌人状态
        self.enemy_lock = False
        self.enemy_distance = 0.0
        self.last_enemy_lock_time = 0
        
        # 游戏状态
        self.is_dead = False
        self.death_count = 0
        self.respawn_time = 0
        self.match_start_time = 0
        self.initial_navigation_complete = False
        self.last_decision_time = 0
        self.last_kill_time = 0
        
        # 中心点占领状态
        self.center_occupied_by = None
        self.center_occupation_start_time = 0
        
        # 云台状态
        self.gimbal_yaw = 0.0
        self.robot_yaw = 0.0
        
        # 探索状态跟踪
        self.explored_points = {}
        self.last_explore_time = 0
        self.current_explore_target = None
        
        # 连接检查
        self.connection_manager.start_connection_check()
        
        # 订阅者
        self.status_sub = self.create_subscription(
            RobotStatus,
            ns_topic(self, 'robot_status'),
            self.status_callback,
            10)

        self.enemy_sub = self.create_subscription(
            EnemyState,
            ns_topic(self, 'enemy_state'),
            self.enemy_state_callback,
            10)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            ns_topic(self, self.map_topic),
            self.map_callback,
            10)

        self.gimbal_sub = self.create_subscription(
            JointState,
            ns_topic(self, 'serial/gimbal_joint_state'),
            self.gimbal_callback,
            10)

        # 服务
        self.set_active_srv = self.create_service(
            SetBool,
            ns_topic(self, 'set_active'),
            self.set_active_callback)

        self.start_match_service = self.create_service(
            SetBool,
            ns_topic(self, 'start_match'),
            self.start_match_callback)
        
        # 定时器
        self.decision_timer = self.create_timer(0.5, self.decision_callback)
        self.nav_check_timer = self.create_timer(0.5, self.nav_check_callback)
        self.center_timer = self.create_timer(1.0, self.center_check_callback)
        self.match_timer = self.create_timer(1.0, self.match_time_check)
        self.debug_timer = self.create_timer(5.0, self.log_current_status)
        
        self.get_logger().info("强化学习导航决策节点已启动")

    def request_restart(self):
        """请求节点重启"""
        if not self.restart_requested and self.restart_count < self.max_restarts:
            self.restart_count += 1
            self.get_logger().error(f"节点异常，将在{self.restart_delay}秒后重启... (重启次数: {self.restart_count}/{self.max_restarts})")
            self.restart_requested = True
            self.restart_timer = self.create_timer(self.restart_delay, self.restart_node)
        elif self.restart_count >= self.max_restarts:
            self.get_logger().fatal(f"达到最大重启次数({self.max_restarts})，停止重启")
            self.shutdown()
    
    def restart_node(self):
        """重启节点"""
        self.q_learning.save_q_table()
        self.get_logger().info("=== 节点重启中 ===")
        self.destroy_node()
        rclpy.shutdown()

    def map_callback(self, msg):
        try:
            self.map_data = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width))
            self.map_metadata = msg.info
            self.get_logger().info(
                f"接收到地图数据: {msg.info.width}x{msg.info.height} "
                f"分辨率: {msg.info.resolution} "
                f"原点: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})",
                throttle_duration_sec=10
            )
            self.exploration.calculate_explore_radii()
        except Exception as e:
            self.get_logger().error(f"处理地图数据失败: {str(e)}")
            self.map_data = None
            self.map_metadata = None

    def enemy_state_callback(self, msg):
        """处理敌人状态更新"""
        if self.is_dead:
            return
            
        lock_changed = (self.enemy_lock != msg.enemy_lock)
        self.enemy_lock = msg.enemy_lock
        self.enemy_distance = msg.enemy_distance
        
        if self.enemy_lock and lock_changed and self.is_active and not self.is_dead:
            self.last_enemy_lock_time = time.time()
            self.get_logger().info("检测到敌人锁定! 立即触发决策")
            self.last_decision_time = 0
            self.decision_callback()
    
    def gimbal_callback(self, msg):
        """处理云台关节状态更新"""
        try:
            if 'gimbal_yaw' in msg.name:
                idx = msg.name.index('gimbal_yaw')
                self.gimbal_yaw = msg.position[idx]
        except Exception as e:
            self.get_logger().warn(f"处理云台状态失败: {str(e)}")

    def log_current_status(self):
        """定期记录当前状态用于调试"""
        status_msgs = [
            f"连接状态: {self.connection_manager.connection_status.name}",
            f"激活状态: {self.is_active}",
            f"比赛状态: {'运行中' if self.is_active and self.match_start_time > 0 else '未开始'}",
            f"初始导航完成: {self.initial_navigation_complete}",
            f"导航中: {self.is_navigating}",
            f"死亡状态: {self.is_dead}",
            f"位置: {self.navigation.get_robot_position() or '未知'}",
            f"中心占领: {self.center_occupied_by or '无'}"
        ]
        self.get_logger().info(" | ".join(status_msgs), throttle_duration_sec=5)

    def enemy_position_determination(self):
        """根据云台朝向和距离计算敌方位置"""
        if not self.enemy_lock or self.enemy_distance <= 0:
            return None
        
        position_yaw = self.navigation.get_robot_position()
        if position_yaw[0] is None:
            return None
            
        (x, y), robot_yaw = position_yaw
        
        # 计算云台在地图坐标系中的绝对朝向
        absolute_yaw = robot_yaw + self.gimbal_yaw
        
        # 计算敌方位置
        enemy_x = x + self.enemy_distance * math.cos(absolute_yaw)
        enemy_y = y + self.enemy_distance * math.sin(absolute_yaw)
        
        return (enemy_x, enemy_y)

    def start_match_callback(self, request, response):
        """启动比赛服务回调"""
        if not request.data:
            response.success = False
            response.message = "无效请求"
            return response
        
        if self.connection_manager.connection_status != ConnectionStatus.CONNECTED:
            response.success = False
            response.message = "未连接到机器人，无法启动比赛"
            self.get_logger().error("启动比赛失败: 未连接到机器人")
            return response
        
        if self.is_dead:
            response.success = False
            response.message = "机器人已死亡，无法启动比赛"
            self.get_logger().error("启动比赛失败: 机器人已死亡")
            return response
            
        # 重置比赛状态
        self.is_active = True
        self.match_start_time = time.time()
        self.self_hp = 100
        self.self_vp = 0
        self.enemy_vp = 0
        self.death_count = 0
        self.is_dead = False
        self.center_occupied_by = None
        self.center_occupation_start_time = 0
        self.initial_navigation_complete = True
        self.last_decision_time = 0
        self.explored_points = {}
        
        # 导航到基地位置
        self.initial_navigation_complete = False
        self.navigation.navigate_to_point(self.base_x, self.base_y)
        
        self.get_logger().info("比赛开始! 机器人已重置")
        
        response.success = True
        response.message = "比赛已开始"
        return response

    def status_callback(self, msg):
        """处理RoboMaster状态更新"""
        if self.is_dead:
            return
            
        self.self_hp = msg.self_hp
        self.self_vp = msg.self_victory_points
        self.enemy_vp = msg.enemy_victory_points
        
        # 检查死亡事件
        if self.self_hp <= 0:
            self.game_logic.handle_death()
            return
            
        # 检查胜利条件
        if self.self_vp >= self.victory_threshold:
            self.get_logger().info("胜利! 我方胜利点达到阈值")
            self.game_logic.end_game(True)
        elif self.enemy_vp >= self.victory_threshold:
            self.get_logger().info("失败! 敌方胜利点达到阈值")
            self.game_logic.end_game(False)

    def set_active_callback(self, request, response):
        """激活/停用决策系统"""
        if request.data:
            if self.connection_manager.connection_status != ConnectionStatus.CONNECTED:
                response.success = False
                response.message = "未连接到机器人，无法激活"
                self.get_logger().error("激活失败: 未连接到机器人")
                return response
            
            if self.is_dead:
                response.success = False
                response.message = "机器人已死亡，无法激活"
                self.get_logger().error("激活失败: 机器人已死亡")
                return response
                
            self.is_active = True
            self.get_logger().info("决策系统激活")
        else:
            self.is_active = False
            self.get_logger().info("决策系统停用")
            self.navigation.cancel_navigation()
        
        response.success = True
        return response

    def center_check_callback(self):
        """检查中心点占领状态并更新胜利点"""
        if self.is_dead or not self.is_active or self.match_start_time == 0:
            return
            
        position_yaw = self.navigation.get_robot_position()
        if position_yaw[0] is None:
            return
            
        (self_x, self_y), _ = position_yaw
        
        # 计算己方到中心点的距离
        self_distance = math.sqrt((self_x - self.center_x)**2 + (self_y - self.center_y)** 2)
        self_in_center = self_distance <= self.center_radius
        
        # 计算敌方到中心点的距离
        enemy_in_center = False
        if self.enemy_lock:
            enemy_pos = self.enemy_position_determination()
            if enemy_pos:
                enemy_x, enemy_y = enemy_pos
                enemy_distance = math.sqrt((enemy_x - self.center_x)**2 + (enemy_y - self.center_y)** 2)
                enemy_in_center = enemy_distance <= self.center_radius
        
        # 更新中心点占领状态
        current_time = time.time()
        
        if self_in_center and not enemy_in_center:
            if self.center_occupied_by != 'self':
                self.center_occupied_by = 'self'
                self.center_occupation_start_time = current_time
                self.get_logger().info("己方占领中心点")
        elif not self_in_center and enemy_in_center:
            if self.center_occupied_by != 'enemy':
                self.center_occupied_by = 'enemy'
                self.center_occupation_start_time = current_time
                self.get_logger().info("敌方占领中心点")
        elif self_in_center and enemy_in_center:
            pass  # 双方都在中心点，不改变状态
        else:
            self.center_occupied_by = None
            self.center_occupation_start_time = 0
        
        # 根据占领状态加分
        if self.center_occupied_by and self.center_occupation_start_time > 0:
            time_elapsed = current_time - self.center_occupation_start_time
            points_earned = int(time_elapsed * self.center_point_rate)
            
            if points_earned > 0:
                if self.center_occupied_by == 'self':
                    self.self_vp += points_earned
                    self.get_logger().info(f"己方占领中心点! 获得{points_earned}点 (总点数: {self.self_vp})")
                else:
                    self.enemy_vp += points_earned
                    self.get_logger().info(f"敌方占领中心点! 敌方获得{points_earned}点 (敌方点数: {self.enemy_vp})")
                
                self.center_occupation_start_time = current_time
                
                # 检查胜利条件
                if self.self_vp >= self.victory_threshold:
                    self.get_logger().info("胜利! 我方胜利点达到阈值")
                    self.game_logic.end_game(True)
                elif self.enemy_vp >= self.victory_threshold:
                    self.get_logger().info("失败! 敌方胜利点达到阈值")
                    self.game_logic.end_game(False)

    def match_time_check(self):
        """检查比赛是否结束"""
        if self.match_start_time == 0:
            return
            
        elapsed = time.time() - self.match_start_time
        remaining = max(0, self.match_duration - elapsed)
        
        if elapsed >= self.match_duration:
            self.game_logic.end_game_by_time()
        elif int(elapsed) % 30 == 0:
            self.get_logger().info(f"比赛剩余时间: {int(remaining)}秒 | 我方点数: {self.self_vp} | 敌方点数: {self.enemy_vp}")

    def nav_check_callback(self,):
        """检查导航状态并处理到达情况"""
        if not self.is_navigating or self.current_target is None:
            return
        
        position_yaw = self.navigation.get_robot_position()
        if position_yaw[0] is None:
            return
        
        (x, y), _ = position_yaw
        tx, ty = self.current_target
        distance = ((x - tx) **2 + (y - ty)** 2) **0.5
        
        if distance < self.position_tolerance:
            self.get_logger().info(f"到达目标点 (距离: {distance:.2f}m)")
            self.is_navigating = False
            self.current_target = None
            self.initial_navigation_complete = True
            
            if self.current_explore_target is not None:
                quantized_point = (
                    round(self.current_explore_target[0]), 
                    round(self.current_explore_target[1])
                )
                self.explored_points[quantized_point] = self.explored_points.get(quantized_point, 0) + 1
                explore_count = self.explored_points[quantized_point]
                self.get_logger().info(f"探索点: ({quantized_point[0]}, {quantized_point[1]}) 探索次数: {explore_count}")
                self.current_explore_target = None

    def decision_callback(self):
        """决策主回调函数"""
        try:
            current_time = time.time()
            
            if current_time - self.last_decision_time < self.decision_interval:
                return
                
            if self.connection_manager.connection_status != ConnectionStatus.CONNECTED:
                status_name = self.connection_manager.connection_status.name
                self.get_logger().warn(
                    f"未连接到机器人 ({status_name})，跳过决策", 
                    throttle_duration_sec=10
                )
                return
                
            if self.is_dead:
                self.get_logger().warn("机器人已死亡，跳过决策", throttle_duration_sec=2)
                return
                
            if not self.is_active:
                self.get_logger().warn("决策系统未激活，跳过决策", throttle_duration_sec=5)
                return
            
            if self.is_navigating:
                position_yaw = self.navigation.get_robot_position()
                if position_yaw[0] and self.current_target:
                    (cx, cy), _ = position_yaw
                    tx, ty = self.current_target
                    distance = math.sqrt((cx - tx)**2 + (cy - ty)** 2)
                    self.get_logger().info(
                        f"导航中，剩余距离: {distance:.2f}m", 
                        throttle_duration_sec=2
                    )
                else:
                    self.get_logger().info("导航中...", throttle_duration_sec=2)
                return
            
            # 获取当前状态并执行决策
            current_state = self.state_processor.get_discrete_state()
            self.get_logger().debug(f"当前离散状态: {current_state}")

            # 验证状态值
            if len(current_state) != 4 or any(not isinstance(x, int) for x in current_state):
                self.get_logger().error(f"无效的离散状态: {current_state}")
                return
            
            action = self.q_learning.choose_action(current_state)
            if action is None:
                return
                
            self.execute_action(action)
            
            # 记录状态和动作用于学习
            self.last_state = current_state
            self.last_action = action
            self.last_decision_time = current_time
            
            # 短暂等待状态变化
            self.create_timer(0.5, self.post_action_learning)
            
        except Exception as e:
            self.get_logger().error(f"决策回调异常: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    def post_action_learning(self):
        """动作执行后的学习过程"""
        try:
            if self.last_state is None or self.last_action is None:
                return
                
            new_state = self.state_processor.get_discrete_state()
            
            # 验证新状态
            if len(new_state) != 4 or any(not isinstance(x, int) for x in new_state):
                self.get_logger().error(f"新状态包含非整数: {new_state}")
                return
                
            # 计算奖励
            reward = self.state_processor.calculate_reward(self.last_state, self.last_action, new_state)
            if isinstance(reward, np.ndarray):
                reward = reward.item()
                
            self.last_reward = reward
            
            # 更新Q表
            self.q_learning.update_q_table(self.last_state, self.last_action, reward, new_state)
            
        except Exception as e:
            self.get_logger().error(f"学习过程异常: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    def execute_action(self, action):
        """执行选定的动作"""
        if self.is_dead or not self.is_active:
            return
            
        position_yaw = self.navigation.get_robot_position()
        if position_yaw[0] is None:
            self.get_logger().warn("无法获取位置，跳过动作执行")
            return
        
        (x, y), _ = position_yaw
        
        if action == Action.PURSUE:
            enemy_pos = self.enemy_position_determination()
            if enemy_pos:
                enemy_x, enemy_y = enemy_pos
                self.navigation.navigate_to_point(enemy_x, enemy_y)
                self.get_logger().info(f"执行动作: 追击敌方 ({enemy_x:.2f}, {enemy_y:.2f})")
            else:
                self.get_logger().warn("未知敌方位置，无法追击")
                # 改为探索动作而不是保持位置
                self.execute_action(Action.EXPLORE_MAP)
        
        elif action == Action.RETURN_TO_BASE:
            self.navigation.navigate_to_point(self.base_x, self.base_y)
            self.get_logger().info("执行动作: 返回基地")
        
        elif action == Action.HOLD_POSITION:
            self.navigation.cancel_navigation()
            self.get_logger().info("执行动作: 保持位置")
            
        elif action == Action.CAPTURE_CENTER:
            self.navigation.navigate_to_point(self.center_x, self.center_y)
            self.get_logger().info("执行动作: 占领中心增益点")
        
        elif action == Action.EXPLORE_MAP:
            explore_point = self.exploration.select_explore_point()
            if explore_point:
                self.navigation.navigate_to_point(explore_point[0], explore_point[1])
                self.current_explore_target = explore_point
                self.get_logger().info(f"执行动作: 探索地图 ({explore_point[0]:.2f}, {explore_point[1]:.2f})")
            else:
                self.get_logger().warn("无法生成探索点，改为保持位置")
                self.execute_action(Action.HOLD_POSITION)
            
        # 撤退动作
        elif action == Action.RETREAT:
            # 撤退到最近的基地或安全点
            position_yaw = self.navigation.get_robot_position()
            if position_yaw[0] is None:
                return
            
            (x, y), _ = position_yaw
            base_distance = math.sqrt((x - self.base_x)**2 + (y - self.base_y)**2)
            center_distance = math.sqrt((x - self.center_x)**2 + (y - self.center_y)**2)
            
            # 选择较近的安全点
            if base_distance < center_distance:
                target_x, target_y = self.base_x, self.base_y
                self.get_logger().info("执行动作: 撤退到基地")
            else:
                target_x, target_y = self.center_x, self.center_y
                self.get_logger().info("执行动作: 撤退到中心点")
                
            self.navigation.navigate_to_point(target_x, target_y)

    def shutdown(self):
        """节点关闭时保存Q表"""
        try:
            self.q_learning.save_q_table()
            self.navigation.cancel_navigation()
            self.get_logger().info("节点关闭")
        except Exception as e:
            self.get_logger().error(f"关闭过程中发生错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    # 使用多线程执行器提高性能
    executor = MultiThreadedExecutor()
    
    max_restarts = 3
    restart_count = 0
    
    while rclpy.ok() and restart_count <= max_restarts:
        try:
            node = ReinforcementLearningNavDecision()
            executor.add_node(node)
            
            try:
                executor.spin()
            except KeyboardInterrupt:
                break
            except Exception as e:
                node.get_logger().error(f"执行异常: {str(e)}")
                node.get_logger().error(traceback.format_exc())
            finally:
                # 确保节点关闭前保存Q表
                node.q_learning.save_q_table()
                executor.remove_node(node)
                node.destroy_node()
                
                # 如果请求重启，则重新初始化节点
                if node.restart_requested and restart_count < max_restarts:
                    restart_count += 1
                    node.get_logger().info(f"重新初始化节点... (重启次数: {restart_count}/{max_restarts})")
                    time.sleep(node.restart_delay)
                else:
                    break
                    
        except Exception as e:
            print(f"严重错误: {str(e)}")
            print(traceback.format_exc())
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()