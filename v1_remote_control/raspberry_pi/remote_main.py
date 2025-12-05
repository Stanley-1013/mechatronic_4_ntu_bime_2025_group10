#!/usr/bin/env python3
"""
main.py - 機電小車主程式
版本: 1.0
日期: 2025-10-31

機器人控制系統主程式進入點
"""

import argparse
import sys
from robot_controller import RobotController


def parse_args():
    """解析命令列參數"""
    parser = argparse.ArgumentParser(
        description='機電小車遙控系統',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  python3 main.py                    # 使用預設設定啟動
  python3 main.py --debug            # 啟用除錯模式
  python3 main.py --no-sensor        # 不顯示感測器資料
        """
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='啟用除錯模式（顯示詳細訊息）'
    )

    parser.add_argument(
        '--no-sensor',
        action='store_true',
        help='不顯示感測器資料'
    )

    parser.add_argument(
        '--version',
        action='version',
        version='機電小車遙控系統 v1.0'
    )

    return parser.parse_args()


def main():
    """主程式"""
    args = parse_args()

    # 應用命令列參數
    if args.debug:
        import config
        config.DEBUG_MODE = True
        config.VERBOSE_SERIAL = True
        print("✅ 除錯模式已啟用\n")

    if args.no_sensor:
        import config
        config.SHOW_SENSOR_DATA = False

    # 顯示歡迎訊息
    print("=" * 60)
    print(" " * 15 + "機電小車遙控系統")
    print(" " * 15 + "Mechatronics Robot Control System")
    print(" " * 22 + "v1.0")
    print("=" * 60)
    print()

    try:
        # 建立並啟動控制器
        controller = RobotController()
        controller.start()

    except KeyboardInterrupt:
        print("\n\n✋ 使用者中止程式")
        sys.exit(0)

    except Exception as e:
        print(f"\n❌ 程式發生錯誤: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
