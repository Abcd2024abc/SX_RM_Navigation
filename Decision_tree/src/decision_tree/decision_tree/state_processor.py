import math
import time
import numpy as np
from .q_learning import Action

class StateProcessor:
    def __init__(self, node):
        self.node = node
        self.reward_pursue = node.get_parameter('reward_pursue').value
        self.reward_return_base = node.get_parameter('reward_return_base').value
        self.reward_hold_position = node.get_parameter('reward_hold_position').value
        self.reward_capture_center = node.get_parameter('reward_capture_center').value
        self.reward_explore_map = node.get_parameter('reward_explore_map').value
        self.reward_retreat = node.get_parameter('reward_retreat').value
        self.center_capture_boost = node.get_parameter('center_capture_boost').value
        self.explore_kill_bonus = node.get_parameter('explore_kill_bonus').value
        self.retreat_success_bonus = node.get_parameter('retreat_success_bonus').value
        self.explore_new_point_reward = node.get_parameter('explore_new_point_reward').value
        self.decision_interval = node.get_parameter('decision_interval').value

    def get_discrete_state(self):
        """将连续状态离散化"""
        # 血量级别 (0-3)
        self_hp_level = 0
        if self.node.self_hp > 70:
            self_hp_level = 3
        elif self.node.self_hp > 50:
            self_hp_level = 2
        elif self.node.self_hp > 30:
            self_hp_level = 1
        
        # 胜利点差异级别 (0-2)
        vp_diff = self.node.self_vp - self.node.enemy_vp
        vp_level = 0
        if vp_diff > 30:
            vp_level = 2
        elif vp_diff >= -10:
            vp_level = 1
        
        # 中心点占领状态 (0-2)
        center_state = 0
        if self.node.center_occupied_by == 'self':
            center_state = 1
        elif self.node.center_occupied_by == 'enemy':
            center_state = 2
        
        # 敌人锁定状态 (0-1)
        enemy_lock_state = 1 if self.node.enemy_lock else 0
        
        return (self_hp_level, vp_level, center_state, enemy_lock_state)

    def calculate_reward(self, prev_state, action, new_state):
        """计算奖励值"""
        # 确保状态值是标量
        if any(isinstance(x, np.ndarray) for x in [prev_state, new_state]):
            self.node.get_logger().warn("状态包含数组值，无法计算奖励")
            return 0
        
        reward = 0
        
        # 动作基础奖励
        if action == Action.PURSUE:
            reward += self.reward_pursue
        elif action == Action.RETURN_TO_BASE:
            reward += self.reward_return_base
        elif action == Action.HOLD_POSITION:
            reward += self.reward_hold_position
        elif action == Action.CAPTURE_CENTER:
            reward += self.reward_capture_center
            if self.node.center_occupied_by == 'self':
                reward += self.center_capture_boost
        elif action == Action.EXPLORE_MAP:
            reward += self.reward_explore_map
            if time.time() - self.node.last_kill_time < self.decision_interval:
                reward += self.explore_kill_bonus
                self.node.get_logger().info(f"探索中击杀! 额外奖励 +{self.explore_kill_bonus:.1f}")
        elif action == Action.RETREAT:
            reward += self.reward_retreat
            position_yaw = self.node.navigation.get_robot_position()
            if position_yaw[0]:
                (x, y), _ = position_yaw
                distance_to_enemy = self.node.enemy_distance
                if distance_to_enemy > 5.0:  # 与敌人保持一定距离
                    reward += self.retreat_success_bonus
                    self.node.get_logger().info(f"成功撤退! 额外奖励 +{self.retreat_success_bonus:.1f}")
        
        # 血量变化奖励 - 强化血量的重要性
        hp_change = self.node.self_hp - prev_state[0]
        if hp_change > 0:
            reward += hp_change * 0.15
        elif hp_change < 0:
            reward += hp_change * 0.25
        
        # 低血量额外惩罚
        if self.node.self_hp < 30:
            reward -= 0.5  # 低血量持续惩罚
        
        # 胜利点奖励
        vp_change = self.node.self_vp - prev_state[1]
        reward += vp_change * 0.5
        
        # 中心点状态奖励
        if self.node.center_occupied_by == 'self':
            reward += 1
        elif self.node.center_occupied_by == 'enemy':
            reward -= 1
        
        # 特别奖励
        if self.node.self_vp >= self.node.victory_threshold:
            reward += 1000
        elif self.node.enemy_vp >= self.node.victory_threshold:
            reward -= 1000
        
        # 死亡惩罚
        if self.node.is_dead:
            reward -= 50
        
        # 探索新区域奖励
        if action == Action.EXPLORE_MAP and self.node.exploration.is_new_explore_point():
            reward += self.explore_new_point_reward
            self.node.get_logger().info(f"发现新区域! 奖励 +{self.explore_new_point_reward:.1f}")
        
        return reward