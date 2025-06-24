import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import serial
from serial.tools import list_ports
import time

def select_port(baudrate):
    ser = serial.Serial()
    ser.baudrate = baudrate    
    ser.timeout = None       # タイムアウトの時間

    ports = list_ports.comports()    # ポートデータを取得
    devices = [info.device for info in ports]

    if len(devices) == 0:
        # シリアル通信できるデバイスが見つからなかった場合
        print("エラー: ポートが見つかりませんでした")
        # 手動でポートを指定してみる
        ser.port = '/dev/ttyS0'  # 必要に応じて適切なデバイス名に変更
        try:
            ser.open()
            print(f"ポート {ser.port} を手動で開きました")
            return ser
        except Exception as e:
            print(f"エラー：手動でポートを開けませんでした。{e}")
            return None
    elif len(devices) == 1:
        print(f"一つだけポートがありました: {devices[0]}")
        ser.port = devices[0]
    else:
        # 複数ポートの場合、選択
        for i in range(len(devices)):
            print(f"input {i:d} open {devices[i]}")
        num = int(input("ポート番号を入力してください: "))
        ser.port = devices[num]
    
    # 開いてみる
    try:
        ser.open()
        print(f"ポート {ser.port} を開きました")
        return ser
    except Exception as e:
        print(f"エラー：ポートを開けませんでした。{e}")
        return None

time.sleep(1)
ser = select_port(115200)
if ser:
    print(f"シリアルポートの設定が完了しました: {ser}")
else:
    print("シリアルポートの設定に失敗しました")



