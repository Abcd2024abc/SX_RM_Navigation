import random
import math
import numpy as np

class Exploration:
    def __init__(self, node):
        self.node = node
        self.explore_min_distance = node.get_parameter('explore_min_distance').value
        self.explore_priority_center = node.get_parameter('explore_priority_center').value
        self.explore_priority_base = node.get_parameter('explore_priority_base').value
        self.explore_priority_enemy = node.get_parameter('explore_priority_enemy').value
        self.position_tolerance = node.get_parameter('position_tolerance').value
        
        # 探索半径
        self.explore_radius_center = 3.0
        self.explore_radius_base = 2.0
        self.explore_radius_enemy = 3.0

    def calculate_explore_radii(self):
        """计算探索半径"""
        if self.node.map_metadata:
            map_width = self.node.map_metadata.width * self.node.map_metadata.resolution
            map_height = self.node.map_metadata.height * self.node.map_metadata.resolution
            diag_length = math.sqrt(map_width**2 + map_height**2)
            base_radius = diag_length * 0.2
            
            self.explore_radius_center = base_radius * 1.2
            self.explore_radius_base = base_radius * 0.8
            self.explore_radius_enemy = base_radius * 1.0
            
            self.node.get_logger().info(
                f"计算探索半径: 中心={self.explore_radius_center:.2f}m, "
                f"基地={self.explore_radius_base:.2f}m, "
                f"敌方={self.explore_radius_enemy:.2f}m"
            )

    def select_explore_point(self):
        """根据优先级选择探索点"""
        position_yaw = self.node.navigation.get_robot_position()
        if position_yaw[0] is None:
            return None
            
        (x, y), _ = position_yaw
        
        # 探索优先级区域
        explore_zones = [
            {
                'center': (self.node.center_x, self.node.center_y),
                'radius': self.explore_radius_center,
                'weight': self.explore_priority_center
            },
            {
                'center': (self.node.base_x, self.node.base_y),
                'radius': self.explore_radius_base,
                'weight': self.explore_priority_base
            },
            {
                'center': (self.node.enemy_base_x, self.node.enemy_base_y),
                'radius': self.explore_radius_enemy,
                'weight': self.explore_priority_enemy
            }
        ]
        
        # 随机选择探索区域（根据权重）
        r = random.random()
        cumulative_weight = 0
        selected_zone = None
        
        for zone in explore_zones:
            cumulative_weight += zone['weight']
            if r <= cumulative_weight:
                selected_zone = zone
                break
        else:
            selected_zone = explore_zones[0]
        
        # 在选定区域内生成随机点
        cx, cy = selected_zone['center']
        radius = selected_zone['radius']
        
        for _ in range(10):
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, radius)
            
            point_x = cx + distance * math.cos(angle)
            point_y = cy + distance * math.sin(angle)
            
            if self.is_valid_explore_point(point_x, point_y, (x, y)):
                return (point_x, point_y)
        
        # 生成附近点
        return self.generate_nearby_point(x, y, radius/2)

    def generate_nearby_point(self, x, y, max_distance=2.0):
        """在当前位置附近生成探索点"""
        for _ in range(5):
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, max_distance)
            
            point_x = x + distance * math.cos(angle)
            point_y = y + distance * math.sin(angle)
            
            if self.is_valid_explore_point(point_x, point_y, (x, y)):
                return (point_x, point_y)
        
        return (x, y)

    def is_valid_explore_point(self, x, y, current_pos):
        """检查探索点是否有效"""
        if not self.is_in_map_boundary(x, y):
            return False
            
        cx, cy = current_pos
        distance = math.sqrt((x - cx)**2 + (y - cy)** 2)
        if distance < self.explore_min_distance:
            return False
            
        return True

    def is_new_explore_point(self):
        """检查是否到达新探索点"""
        if self.node.current_explore_target is None:
            return False
            
        position_yaw = self.node.navigation.get_robot_position()
        if position_yaw[0] is None:
            return False
            
        (x, y), _ = position_yaw
        tx, ty = self.node.current_explore_target
        
        distance = math.sqrt((x - tx)**2 + (y - ty)** 2)
        if distance < self.position_tolerance:
            quantized_point = (round(tx, 1), round(ty, 1))
            if self.node.explored_points.get(quantized_point, 0) == 0:
                return True
                
        return False

    def is_in_map_boundary(self, x, y):
        """检查位置是否在地图边界内（确保输入是标量）"""
        # 确保输入是标量
        if isinstance(x, np.ndarray):
            x = x.item()
        if isinstance(y, np.ndarray):
            y = y.item()
        
        # 如果地图元数据不存在，返回 True（假设位置有效）
        if not hasattr(self.node, 'map_metadata') or self.node.map_metadata is None:
            return True
            
        map_info = self.node.map_metadata
        
        min_x = map_info.origin.position.x
        min_y = map_info.origin.position.y
        max_x = min_x + map_info.width * map_info.resolution
        max_y = min_y + map_info.height * map_info.resolution
        
        x_valid = min_x <= x <= max_x
        y_valid = min_y <= y <= max_y
        
        # 处理可能的布尔数组
        if isinstance(x_valid, np.ndarray):
            x_valid = x_valid.all()
        if isinstance(y_valid, np.ndarray):
            y_valid = y_valid.all()
            
        return x_valid and y_valid