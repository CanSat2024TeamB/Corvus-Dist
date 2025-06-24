import serial
import time

def send_and_receive(ser, data, wait_time=2):
    try:
        ser.write(data.encode('ascii'))  # データをASCIIエンコードして送信
        print('sent')
        ser.flush()
        print('wait start')
        time.sleep(wait_time)  # 受信するための待機時間を設定
        response = ser.read_all()

        print(f"Raw response (bytes): {response}")

        # 試しに複数のエンコーディングでデコードを試みる
        try:
            response_decoded = response.decode('utf-8').strip()
            print(f"UTF-8 decoded response: {response_decoded}")
        except UnicodeDecodeError:
            try:
                response_decoded = response.decode('latin-1').strip()
                print(f"Latin-1 decoded response: {response_decoded}")
            except UnicodeDecodeError:
                try:
                    response_decoded = response.decode('ISO-8859-1').strip()
                    print(f"ISO-8859-1 decoded response: {response_decoded}")
                except UnicodeDecodeError:
                    try:
                        response_decoded = response.decode('ascii').strip()
                        print(f"ASCII decoded response: {response_decoded}")
                    except UnicodeDecodeError:
                        print("すべてのデコード試行が失敗しました")
                        response_decoded = ""

        return response_decoded
    except Exception as e:
        print(f"通信エラー: {e}")
        return ""

def main():
    try:
        # シリアルポートの設定
        ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)  # ボーレートとその他の設定を追加
        if ser.is_open:
            print(f"シリアルポート {ser.port} を開きました")
        else:
            print("シリアルポートを開けませんでした")
            return
    except Exception as e:
        print(f"シリアルポートのオープンエラー: {e}")
        return

    # データの送受信
    for i in range(10):
        data_to_send = 'p2p tx 1234\n\r'  # 送信データは文字列のまま
        print(f"送信したデータ: {data_to_send}")
        response = send_and_receive(ser, data_to_send)
        print(f"受信したデータ: {response}")
        time.sleep(2)

    # シリアルポートを閉じる
    try:
        ser.close()
        print("シリアルポートを閉じました")
    except Exception as e:
        print(f"シリアルポートのクローズエラー: {e}")

if __name__ == "__main__":
    main()
