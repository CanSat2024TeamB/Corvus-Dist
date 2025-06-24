from pathlib import Path
import datetime
import os

class Logger:
    def __init__(self, dir):
        # 動的にログファイルのパスを生成
        self.dir = dir
        self.path = str(Path(self.dir).joinpath(f"log_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.txt"))
        print(f"Logs will be written to the file, {self.path}")
        self.create_file()

    def create_file(self) -> bool:
        # ディレクトリが存在しない場合は作成する
        os.makedirs(self.dir, exist_ok=True)
        # ファイルを作成し、初期メッセージを書き込みます。
        self.write("Log file created")
        return True

    def write(self, *msg: str, no_print = False) -> bool:
        string = str(msg[0])
        if len(msg) > 1:
            for i in range(1, len(msg)):
                string += " " + str(msg[i])

        if not no_print:
            print(string)
        
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.path, 'a', encoding="UTF-8") as f:
            f.write(f"{timestamp} {string}\n")
        return True

