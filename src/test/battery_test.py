import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController

async def main():
    dronecontroller = DroneController()
    
    # 接続処理を非同期で実行
    await dronecontroller.connect()
    
    # バッテリー監視タスクを開始
    battery_task = asyncio.create_task(dronecontroller.battery_watch.invoke_loop())
    await asyncio.sleep(5)
    
    try:
        while True:
            # バッテリー情報を取得して表示
            print("Voltage:", dronecontroller.battery_watch.voltage_v())
            print("Remaining Percent:", dronecontroller.battery_watch.remaining_percent())
            print("Current Battery A:", dronecontroller.battery_watch.current_battery_a())
            
            # 1秒待機
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        # タスクがキャンセルされた場合の処理
        print("Battery watch task was cancelled")

asyncio.run(main())
