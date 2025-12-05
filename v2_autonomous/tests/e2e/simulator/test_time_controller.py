"""
時序控制器測試套件

測試項目:
1. 基本時間步進正確性
2. 阻塞發生機率符合設定
3. 長時間運行的累積誤差
4. 感測器延遲整合
"""

import unittest
import statistics
from time_controller import TimeController, TimeStats


class TestTimeControllerBasic(unittest.TestCase):
    """基本時間步進測試"""

    def test_basic_time_stepping(self):
        """測試基本時間步進 - 無阻塞"""
        controller = TimeController(blocking_probability=0.0)

        # 執行 50 步 (應為 1 秒)
        for _ in range(50):
            dt = controller.step()
            self.assertAlmostEqual(dt, 0.02, places=6)

        # 驗證累積時間
        self.assertAlmostEqual(controller.get_time(), 1.0, places=6)
        self.assertEqual(controller.get_time_ms(), 1000)
        self.assertEqual(controller.get_step_count(), 50)

    def test_custom_base_dt(self):
        """測試自訂基本時間步長"""
        dt_custom = 0.01  # 10ms (100Hz)
        controller = TimeController(base_dt=dt_custom, blocking_probability=0.0)

        for _ in range(100):
            dt = controller.step()
            self.assertAlmostEqual(dt, dt_custom, places=6)

        # 100 步 * 10ms = 1 秒
        self.assertAlmostEqual(controller.get_time(), 1.0, places=6)

    def test_time_accumulation(self):
        """測試時間累積"""
        controller = TimeController(blocking_probability=0.0)
        expected_time = 0.0

        for i in range(100):
            dt = controller.step()
            expected_time += dt
            self.assertAlmostEqual(controller.get_time(), expected_time, places=9)

    def test_step_count_increment(self):
        """測試步數計數"""
        controller = TimeController()

        for expected_count in range(1, 51):
            controller.step()
            self.assertEqual(controller.get_step_count(), expected_count)


class TestBlockingBehavior(unittest.TestCase):
    """阻塞行為測試"""

    def test_blocking_probability(self):
        """測試阻塞機率符合設定"""
        # 設定 20% 阻塞機率
        blocking_prob = 0.2
        controller = TimeController(blocking_probability=blocking_prob)

        num_steps = 1000
        blocked_count = 0

        for _ in range(num_steps):
            initial_time = controller.current_time
            dt = controller.step()

            # 如果 dt > base_dt，表示發生了阻塞
            if dt > controller.base_dt + 1e-9:
                blocked_count += 1

        # 統計實際阻塞機率
        actual_prob = blocked_count / num_steps

        # 允許偏差 ±50% (因為是隨機)
        expected_min = blocking_prob * 0.5
        expected_max = blocking_prob * 1.5

        self.assertGreaterEqual(actual_prob, expected_min,
                               f"阻塞機率過低: {actual_prob} < {expected_min}")
        self.assertLessEqual(actual_prob, expected_max,
                            f"阻塞機率過高: {actual_prob} > {expected_max}")

    def test_blocking_delay_range(self):
        """測試阻塞延遲在指定範圍內"""
        max_blocking = 0.05  # 50ms
        controller = TimeController(
            blocking_probability=0.5,
            max_blocking_time=max_blocking
        )

        for _ in range(200):
            dt = controller.step()
            blocking_delay = dt - controller.base_dt

            # 要麼沒有阻塞 (blocking_delay ≈ 0)，要麼在範圍內
            self.assertTrue(
                blocking_delay < 1e-9 or (0 <= blocking_delay <= max_blocking + 1e-9),
                f"阻塞延遲超出範圍: {blocking_delay}"
            )

    def test_blocking_stats(self):
        """測試阻塞統計準確"""
        controller = TimeController(
            blocking_probability=0.1,
            max_blocking_time=0.05
        )

        for _ in range(500):
            controller.step()

        stats = controller.get_stats()

        # 驗證統計資訊
        self.assertEqual(stats.total_steps, 500)
        self.assertAlmostEqual(
            stats.total_time,
            controller.get_time(),
            places=9
        )
        self.assertEqual(stats.blocked_steps, controller._blocked_steps)
        self.assertAlmostEqual(
            stats.total_blocking_time,
            controller._total_blocking_time,
            places=9
        )

    def test_blocking_distribution_uniform(self):
        """測試均勻分布阻塞延遲"""
        controller = TimeController(
            blocking_probability=0.5,
            max_blocking_time=0.1,
            blocking_distribution="uniform"
        )

        delays = []
        for _ in range(200):
            dt = controller.step()
            delay = dt - controller.base_dt
            if delay > 1e-9:
                delays.append(delay)

        # 驗證延遲範圍和統計特性
        if len(delays) > 10:
            avg_delay = statistics.mean(delays)
            # 均勻分布的平均應接近 max/2 = 50ms
            expected_avg = 0.1 / 2
            # 允許 ±40% 偏差
            self.assertGreater(avg_delay, expected_avg * 0.6)
            self.assertLess(avg_delay, expected_avg * 1.4)

    def test_blocking_distribution_gaussian(self):
        """測試高斯分布阻塞延遲"""
        controller = TimeController(
            blocking_probability=0.5,
            max_blocking_time=0.1,
            blocking_distribution="gaussian"
        )

        delays = []
        for _ in range(200):
            dt = controller.step()
            delay = dt - controller.base_dt
            if delay > 1e-9:
                delays.append(delay)

        # 驗證延遲範圍 (高斯分布應集中在低延遲)
        if len(delays) > 10:
            avg_delay = statistics.mean(delays)
            # 高斯分布平均應 < 均勻分布 (低延遲佔多數)
            self.assertLess(avg_delay, 0.05)


class TestSensorIntegration(unittest.TestCase):
    """感測器延遲整合測試"""

    def test_sensor_reading_registration(self):
        """測試感測器讀取註冊"""
        controller = TimeController(sensor_delay=0.06)

        # 在 t=0 註冊感測值
        controller.register_sensor_reading("sensor_1", 42.0)

        # 感測值應在 t=0.06s 後可讀
        self.assertEqual(controller.get_pending_sensor_count(), 1)

        # 推進時間到 0.04s
        for _ in range(2):
            controller.step()

        # 仍未可讀
        self.assertEqual(controller.get_pending_sensor_count(), 1)
        available = controller.get_sensor_readings_available()
        self.assertEqual(len(available), 0)

        # 推進到 0.08s (超過延遲)
        for _ in range(4):
            controller.step()

        # 現在應可讀
        available = controller.get_sensor_readings_available()
        self.assertEqual(len(available), 1)
        self.assertEqual(available[0]['sensor_id'], 'sensor_1')
        self.assertEqual(available[0]['value'], 42.0)
        self.assertAlmostEqual(available[0]['register_time'], 0.0, places=6)

    def test_multiple_sensor_readings(self):
        """測試多個感測器讀取"""
        controller = TimeController(sensor_delay=0.06)

        # 在不同時間註冊感測值
        for i in range(5):
            controller.step()
            controller.register_sensor_reading(f"sensor_{i}", float(i * 10))

        # 推進時間
        for _ in range(20):
            controller.step()

        # 檢查可讀感測值
        available = controller.get_sensor_readings_available()
        self.assertEqual(len(available), 5)

    def test_sensor_queue_cleanup(self):
        """測試感測器隊列清理"""
        controller = TimeController(sensor_delay=0.02)

        # 註冊 10 個感測值
        for _ in range(10):
            controller.register_sensor_reading("sensor", 1.0)
            controller.step()

        # 推進至全部可讀
        for _ in range(20):
            controller.step()

        # 讀取所有感測值
        available = controller.get_sensor_readings_available()
        self.assertEqual(len(available), 10)

        # 隊列應清空
        self.assertEqual(controller.get_pending_sensor_count(), 0)

    def test_sensor_delay_customization(self):
        """測試自訂感測器延遲"""
        custom_delay = 0.1  # 100ms
        controller = TimeController(sensor_delay=custom_delay)

        controller.register_sensor_reading("sensor", 99.9)

        # 推進 4 步 (80ms) - 仍未可讀
        for _ in range(4):
            controller.step()

        available = controller.get_sensor_readings_available()
        self.assertEqual(len(available), 0)

        # 再推進 1 步 (100ms) - 現在可讀
        controller.step()
        available = controller.get_sensor_readings_available()
        self.assertEqual(len(available), 1)


class TestLongRunningAccuracy(unittest.TestCase):
    """長時間運行的累積誤差測試"""

    def test_long_run_without_blocking(self):
        """測試長時間運行（無阻塞）的累積誤差"""
        controller = TimeController(blocking_probability=0.0)

        # 執行 10000 步 (200 秒)
        num_steps = 10000
        for _ in range(num_steps):
            controller.step()

        # 預期時間
        expected_time = num_steps * controller.base_dt

        # 應該完全精確
        self.assertAlmostEqual(controller.get_time(), expected_time, places=9)

    def test_long_run_with_blocking(self):
        """測試長時間運行（有阻塞）的時間追蹤"""
        controller = TimeController(blocking_probability=0.1)

        accumulated_dt = 0.0
        for _ in range(1000):
            dt = controller.step()
            accumulated_dt += dt

        # 驗證時間累積精確
        self.assertAlmostEqual(
            controller.get_time(),
            accumulated_dt,
            places=9
        )

    def test_time_stats_accuracy(self):
        """測試時間統計準確性"""
        controller = TimeController(
            blocking_probability=0.15,
            max_blocking_time=0.08
        )

        # 執行多步
        for _ in range(500):
            controller.step()

        stats = controller.get_stats()

        # 驗證統計一致性
        self.assertEqual(stats.total_steps, 500)
        self.assertGreater(stats.total_time, 500 * controller.base_dt)
        self.assertGreater(stats.blocked_steps, 0)
        self.assertGreater(stats.total_blocking_time, 0)
        self.assertGreaterEqual(stats.min_dt, controller.base_dt)
        self.assertLessEqual(stats.min_dt, stats.max_dt)
        self.assertGreaterEqual(stats.avg_dt, controller.base_dt)

    def test_reset_clears_stats(self):
        """測試重設清空統計"""
        controller = TimeController()

        # 執行幾步
        for _ in range(100):
            controller.step()

        self.assertGreater(controller.get_step_count(), 0)
        self.assertGreater(controller.get_time(), 0)

        # 重設
        controller.reset()

        self.assertEqual(controller.get_step_count(), 0)
        self.assertEqual(controller.get_time(), 0.0)
        self.assertEqual(controller.get_time_ms(), 0)

        stats = controller.get_stats()
        self.assertEqual(stats.total_steps, 0)
        self.assertEqual(stats.total_time, 0.0)


class TestDynamicParameterAdjustment(unittest.TestCase):
    """動態參數調整測試"""

    def test_adjust_blocking_probability(self):
        """測試動態調整阻塞機率"""
        controller = TimeController(blocking_probability=0.0)

        # 執行 10 步（無阻塞）
        for _ in range(10):
            dt = controller.step()
            self.assertAlmostEqual(dt, controller.base_dt, places=6)

        # 調整阻塞機率為 100%
        controller.set_blocking_params(probability=1.0)

        # 後續步驟應全部有阻塞
        blocked_count = 0
        for _ in range(20):
            dt = controller.step()
            if dt > controller.base_dt + 1e-9:
                blocked_count += 1

        self.assertEqual(blocked_count, 20)

    def test_adjust_max_blocking_time(self):
        """測試動態調整最大阻塞時間"""
        controller = TimeController(
            blocking_probability=1.0,
            max_blocking_time=0.05
        )

        # 執行 10 步，記錄阻塞
        delays_before = []
        for _ in range(10):
            dt = controller.step()
            delay = dt - controller.base_dt
            delays_before.append(delay)

        # 調整最大阻塞時間為 0.02
        controller.set_blocking_params(max_time=0.02)

        # 後續阻塞應更小
        delays_after = []
        for _ in range(10):
            dt = controller.step()
            delay = dt - controller.base_dt
            delays_after.append(delay)
            self.assertLessEqual(delay, 0.02 + 1e-9)

    def test_adjust_blocking_distribution(self):
        """測試動態調整阻塞分布"""
        controller = TimeController(
            blocking_probability=0.5,
            blocking_distribution="uniform"
        )

        # 改為高斯分布
        controller.set_blocking_params(distribution="gaussian")

        # 驗證參數已改變
        self.assertEqual(controller.blocking_distribution, "gaussian")

        # 執行以驗證行為改變
        delays = []
        for _ in range(100):
            dt = controller.step()
            delay = dt - controller.base_dt
            if delay > 1e-9:
                delays.append(delay)

        # 高斯分布應集中在較小的延遲
        if len(delays) > 5:
            avg_delay = statistics.mean(delays)
            self.assertLess(avg_delay, 0.05)  # 應小於最大的一半


if __name__ == '__main__':
    unittest.main()
