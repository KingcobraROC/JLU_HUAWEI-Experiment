import collections

class CommandFilter:
    def __init__(self, window_size=5):
        """初始化指令滤波器"""
        self.window_size = window_size
        self.car_command_history = collections.deque(maxlen=window_size)
    
    def update(self, car_command):
        """更新指令并返回滤波后的结果"""
        self.car_command_history.append(car_command)
        counter = collections.Counter(self.car_command_history)
        most_common = counter.most_common(1) 
        # 判断是否满足滤波条件：最频繁car_command的出现次数 >= 窗口大小的一半 + 1
        if most_common[0][1] >= self.window_size // 2 + 1:
            # 条件满足，返回出现最频繁的car_command
            return most_common[0][0]
        else:
            # 条件不满足，返回最近的一个car_command
            return self.car_command_history[-1] if len(self.car_command_history) > 0 else "stop"