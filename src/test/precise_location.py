import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from pathlib import Path
from statistics import mean, stdev
from drone.drone_controller import DroneController

async def main():
    dronecontroller = DroneController()
    
    # 接続処理を非同期で実行
    await dronecontroller.connect()
    
    # バッテリー監視タスクを開始
    GPS_TASK = asyncio.create_task(dronecontroller.gps_handler.invoke_loop())
    await asyncio.sleep(5)
    
    lat_list = []
    lon_list = []
    AMSL_list = []

    for i in range(100):
        # GPSデータを取得してリストに追加
        lat = dronecontroller.position_manager.adjusted_coordinates_lat()
        lon = dronecontroller.position_manager.adjusted_coordinates_lon()
        AMSL = dronecontroller.position_manager.adjusted_coordinates_AMSL()
        lat_list.append(lat)
        lon_list.append(lon)
        AMSL_list.append(AMSL)

        print(f"Latitude: {lat}, Longitude: {lon}, AMSL: {AMSL}")
        await asyncio.sleep(1)

    # 平均値を計算
    lat_mean = mean(lat_list)
    lon_mean = mean(lon_list)
    AMSL_mean = mean(AMSL_list)

    # 標準偏差を計算
    lat_stdev = stdev(lat_list)
    lon_stdev = stdev(lon_list)
    AMSL_stdev = stdev(AMSL_list)

    # 結果を出力
    print(f"Latitude Mean: {lat_mean}, Latitude Standard Deviation: {lat_stdev}")
    print(f"Longitude Mean: {lon_mean}, Longitude Standard Deviation: {lon_stdev}")
    print(f"AMSL Mean: {AMSL_mean}, AMSL Standard Deviation: {AMSL_stdev}")

asyncio.run(main())