"""
時序控制器 - 模擬控制迴圈阻塞

支援 50Hz 基本頻率，隨機延遲 0-100ms，感測器延遲 60ms 整合。
"""

import random
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class TimeStats:
    """時間統計資訊"""
    total_steps: int
    total_time: float
    blocked_steps: int
    total_blocking_time: float
    min_dt: float
    max_dt: float
    avg_dt: float


class TimeController:
    """
    時序控制器 - 模擬實時控制系統的時間推進

    功能:
    - 基本 50Hz (dt=20ms) 時間步進
    - 隨機阻塞延遲模擬 (0-100ms)
    - 感測器延遲追蹤 (60ms)
    - 累積時間和統計
    """

    def __init__(
        self,
        base_dt: float = 0.02,
        blocking_probability: float = 0.05,
        max_blocking_time: float = 0.1,
        sensor_delay: float = 0.06,
        blocking_distribution: str = "uniform"
    ):
        """
        初始化時序控制器

        Args:
            base_dt: 基本時間步長 (秒)，預設 0.02s (50Hz)
            blocking_probability: 阻塞發生機率 (0-1)，預設 5%
            max_blocking_time: 最大阻塞時間 (秒)，預設 0.1s
            sensor_delay: 感測器延遲 (秒)，預設 0.06s
            blocking_distribution: 阻塞延遲分布 ("uniform" 或 "gaussian")
        """
        self.base_dt = base_dt
        self.blocking_probability = blocking_probability
        self.max_blocking_time = max_blocking_time
        self.sensor_delay = sensor_delay
        self.blocking_distribution = blocking_distribution

        # 時間追蹤
        self.current_time = 0.0
        self.step_count = 0

        # 統計
        self._blocked_steps = 0
        self._total_blocking_time = 0.0
        self._min_dt = float('inf')
        self._max_dt = 0.0
        self._dt_sum = 0.0

        # 感測器延遲隊列 - (讀取時間, 感測值)
        self._sensor_queue = []

    def step(self) -> float:
        """
        執行一步時間推進

        Returns:
            實際經過的時間 (秒)，可能包含阻塞延遲
        """
        dt = self.base_dt
        is_blocked = False

        # 模擬控制迴圈阻塞
        if random.random() < self.blocking_probability:
            blocking = self._generate_blocking_delay()
            dt += blocking
            self._blocked_steps += 1
            self._total_blocking_time += blocking
            is_blocked = True

        # 更新統計
        self.current_time += dt
        self.step_count += 1
        self._min_dt = min(self._min_dt, dt)
        self._max_dt = max(self._max_dt, dt)
        self._dt_sum += dt

        return dt

    def _generate_blocking_delay(self) -> float:
        """
        生成阻塞延遲

        Returns:
            阻塞延遲時間 (秒)
        """
        if self.blocking_distribution == "gaussian":
            # 正態分布：平均阻塞較小，偶爾突發長延遲
            mean = self.max_blocking_time * 0.2
            std = self.max_blocking_time * 0.15
            delay = abs(random.gauss(mean, std))
            return min(delay, self.max_blocking_time)
        else:
            # 均勻分布：阻塞時間均勻分布在 0-max_blocking_time
            return random.uniform(0, self.max_blocking_time)

    def get_time(self) -> float:
        """
        獲取當前累積時間

        Returns:
            累積時間 (秒)
        """
        return self.current_time

    def get_time_ms(self) -> int:
        """
        獲取當前累積時間（毫秒）

        Returns:
            累積時間 (毫秒)
        """
        return int(self.current_time * 1000)

    def get_step_count(self) -> int:
        """
        獲取執行的時間步數

        Returns:
            時間步數
        """
        return self.step_count

    def register_sensor_reading(
        self,
        sensor_id: str,
        value: float,
        current_time: Optional[float] = None
    ) -> None:
        """
        註冊感測器讀取

        實際讀取時間 = 當前時間 + sensor_delay

        Args:
            sensor_id: 感測器 ID
            value: 感測值
            current_time: 註冊時間 (預設為當前時間)
        """
        if current_time is None:
            current_time = self.current_time

        # 感測值在當前時間讀取，但在 sensor_delay 後才返回
        read_time = current_time + self.sensor_delay
        self._sensor_queue.append({
            'sensor_id': sensor_id,
            'value': value,
            'register_time': current_time,
            'read_time': read_time
        })

    def get_sensor_readings_available(self) -> list:
        """
        獲取可用的感測器讀取 (延遲已過期)

        Returns:
            可用感測器讀取列表
        """
        available = []
        remaining = []

        for reading in self._sensor_queue:
            if reading['read_time'] <= self.current_time:
                available.append(reading)
            else:
                remaining.append(reading)

        self._sensor_queue = remaining
        return available

    def get_pending_sensor_count(self) -> int:
        """
        獲取待讀的感測器數量

        Returns:
            待讀感測器數
        """
        return len(self._sensor_queue)

    def get_stats(self) -> TimeStats:
        """
        獲取時間統計

        Returns:
            TimeStats 物件
        """
        avg_dt = self._dt_sum / self.step_count if self.step_count > 0 else 0.0

        return TimeStats(
            total_steps=self.step_count,
            total_time=self.current_time,
            blocked_steps=self._blocked_steps,
            total_blocking_time=self._total_blocking_time,
            min_dt=self._min_dt if self._min_dt != float('inf') else 0.0,
            max_dt=self._max_dt,
            avg_dt=avg_dt
        )

    def reset(self) -> None:
        """重設控制器"""
        self.current_time = 0.0
        self.step_count = 0
        self._blocked_steps = 0
        self._total_blocking_time = 0.0
        self._min_dt = float('inf')
        self._max_dt = 0.0
        self._dt_sum = 0.0
        self._sensor_queue.clear()

    def set_blocking_params(
        self,
        probability: Optional[float] = None,
        max_time: Optional[float] = None,
        distribution: Optional[str] = None
    ) -> None:
        """
        動態調整阻塞參數

        Args:
            probability: 新的阻塞機率
            max_time: 新的最大阻塞時間
            distribution: 新的分布方式
        """
        if probability is not None:
            self.blocking_probability = max(0.0, min(1.0, probability))
        if max_time is not None:
            self.max_blocking_time = max(0.0, max_time)
        if distribution is not None and distribution in ("uniform", "gaussian"):
            self.blocking_distribution = distribution
