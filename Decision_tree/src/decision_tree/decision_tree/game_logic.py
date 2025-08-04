import time

class GameLogic:
    def __init__(self, node):
        self.node = node
        self.respawn_time_base = node.get_parameter('respawn_time_base').value
        self.kill_points = node.get_parameter('kill_points').value
        self.death_points = node.get_parameter('death_points').value
        self.victory_threshold = node.get_parameter('victory_points_threshold').value
        self.match_duration = node.get_parameter('match_duration').value

    def handle_death(self):
        """处理机器人死亡事件"""
        self.node.is_dead = True
        self.node.death_count += 1
        self.node.respawn_time = self.respawn_time_base + (self.node.death_count - 1) * 10
        
        # 取消所有导航
        self.node.navigation.cancel_navigation()
        
        # 给敌方加分
        self.node.enemy_vp += self.death_points
        self.node.get_logger().info(f"被击杀! 敌方获得{self.death_points}点. 复活时间: {self.node.respawn_time}秒")
        
        # 使用ROS 2定时器实现复活
        self.node.create_timer(self.node.respawn_time, self.respawn)
        
    def handle_kill(self):
        """处理击杀敌方事件"""
        self.node.self_vp += self.kill_points
        self.node.last_kill_time = time.time()
        self.node.get_logger().info(f"击杀敌方! 获得{self.kill_points}点")

    def respawn(self):
        """复活机器人"""
        self.node.is_dead = False
        self.node.self_hp = 100
        self.node.get_logger().info("已复活，返回基地")
        
        # 返回基地
        self.node.navigation.navigate_to_point(self.node.base_x, self.node.base_y)

    def end_game_by_time(self):
        """时间结束处理游戏结果"""
        if self.node.self_vp > self.node.enemy_vp:
            self.node.get_logger().info(f"时间到! 胜利! 我方点数: {self.node.self_vp} vs 敌方点数: {self.node.enemy_vp}")
            self.end_game(True)
        elif self.node.enemy_vp > self.node.self_vp:
            self.node.get_logger().info(f"时间到! 失败! 敌方点数: {self.node.enemy_vp} vs 我方点数: {self.node.self_vp}")
            self.end_game(False)
        else:
            self.node.get_logger().info(f"时间到! 平局! 双方点数: {self.node.self_vp}")
            self.end_game(None)

    def end_game(self, is_win):
        """结束游戏"""
        self.node.is_active = False
        self.node.navigation.cancel_navigation()
        
        if is_win is True:
            self.node.get_logger().info("游戏结束 - 胜利!")
        elif is_win is False:
            self.node.get_logger().info("游戏结束 - 失败!")
        else:
            self.node.get_logger().info("游戏结束 - 平局!")