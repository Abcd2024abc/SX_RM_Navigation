import numpy as np
import pickle
import os
import random
from enum import Enum

class Action(Enum):
    PURSUE = 0
    RETURN_TO_BASE = 1
    HOLD_POSITION = 2
    CAPTURE_CENTER = 3
    EXPLORE_MAP = 4
    RETREAT = 5

class QLearning:
    def __init__(self, node):
        self.node = node
        self.q_table_path = node.get_parameter('q_table_path').value
        self.learning_rate = node.get_parameter('learning_rate').value
        self.discount_factor = node.get_parameter('discount_factor').value
        self.exploration_rate = node.get_parameter('exploration_rate').value
        self.q_table = self.load_q_table()

    def load_q_table(self):
        """加载或初始化Q表"""
        expected_shape = (4, 3, 3, 2, len(Action))
        
        # 确保目录存在
        os.makedirs(os.path.dirname(self.q_table_path), exist_ok=True)
        
        if os.path.exists(self.q_table_path):
            try:
                with open(self.q_table_path, 'rb') as f:
                    self.node.get_logger().info(f"从 {self.q_table_path} 加载Q表")
                    q_table = pickle.load(f)
                    
                    if q_table.shape != expected_shape:
                        self.node.get_logger().warn(
                            f"Q表结构不正确! 期望 {expected_shape}, 实际 {q_table.shape}. "
                            "创建新的Q表"
                        )
                        return np.zeros(expected_shape, dtype=np.float64)
                    
                    if not np.isscalar(q_table[0,0,0,0,0]):
                        self.node.get_logger().warn("Q表包含非标量元素，重新初始化")
                        return np.zeros(expected_shape, dtype=np.float64)
                        
                    return q_table.astype(np.float64)
            except Exception as e:
                self.node.get_logger().error(f"加载Q表失败: {str(e)}")
        
        self.node.get_logger().info("创建新的Q表")
        return np.zeros(expected_shape, dtype=np.float64)

    def save_q_table(self):
        """保存Q表"""
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(self.q_table_path), exist_ok=True)
            
            with open(self.q_table_path, 'wb') as f:
                pickle.dump(self.q_table, f)
            self.node.get_logger().info(f"Q表已保存到 {self.q_table_path}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"保存Q表失败: {str(e)}")
            return False

    def choose_action(self, state):
        """根据当前状态选择动作"""
        if self.node.is_dead or not self.node.is_active:
            return None
        
        # 探索-利用权衡
        if random.random() < self.exploration_rate:
            return random.choice(list(Action))
        
        # 利用Q表选择最佳动作
        state_index = state
        q_values = self.q_table[state_index]
        return Action(np.argmax(q_values))

    def update_q_table(self, state, action, reward, next_state):
        """更新Q表"""
        try:
            # 确保状态是整数元组
            state_idx = tuple(int(x) for x in state)
            next_state_idx = tuple(int(x) for x in next_state)
            action_idx = int(action.value)
            
            # 检查状态索引边界
            if len(state_idx) != 4 or len(next_state_idx) != 4:
                self.node.get_logger().warn(f"状态维度错误: {len(state_idx)} (应为4)")
                return
                
            # 检查每个维度是否在有效范围内
            valid_ranges = [(0, 3), (0, 2), (0, 2), (0, 1)]
            for i, (idx, (min_val, max_val)) in enumerate(zip(state_idx, valid_ranges)):
                if idx < min_val or idx > max_val:
                    self.node.get_logger().warn(f"状态索引{i}超出范围: {idx} (应在{min_val}-{max_val})")
                    return
            
            # 检查动作索引
            if action_idx < 0 or action_idx >= len(Action):
                self.node.get_logger().warn(f"无效的动作索引: {action_idx}")
                return
        
            # 获取当前Q值
            current_q = self.q_table[state_idx + (action_idx,)]
            if isinstance(current_q, np.ndarray):
                current_q = current_q.item()
                
            # 获取下一状态最大Q值
            next_q_values = self.q_table[next_state_idx]
            max_next_q = np.max(next_q_values)
            if isinstance(max_next_q, np.ndarray):
                max_next_q = max_next_q.item()
        
            # 更新Q值
            new_q = current_q + self.learning_rate * (reward + self.discount_factor * max_next_q - current_q)
            self.q_table[state_idx + (action_idx,)] = new_q
            
        except IndexError as e:
            self.node.get_logger().error(f"索引错误: {str(e)}")
            self.node.get_logger().error(f"状态: {state}, 动作: {action}, Q表形状: {self.q_table.shape}")
        except Exception as e:
            self.node.get_logger().error(f"更新Q表失败: {str(e)}")
            self.node.get_logger().error(f"状态: {state}, 动作: {action}, 奖励: {reward}, 下一状态: {next_state}")