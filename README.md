# Corvus

## ファイル構成
### /drone  
  ドローン全体の制御関係
### /sensor  
  各種センサーのハンドラ
### /flight  
  pixhawkの離陸・飛行・着陸に関係するやつ
### /control  
  pixhawkの一般的な姿勢・位置制御に関するやつ
### /config  
  コンフィグの読み書き
### /logger  
  ログ関係はここ
  
## Reference
### /drone
#### ・DroneController()
    drone() -> System  
      return: mavsdkのドローンオブジェクト  
      
    async set_up() -> None  
      ドローンを飛ばす前の設定一覧  
      
    async connect() -> bool  
      Pixhawkと接続する  
      return: 成否  
      
    async arm() -> bool  
      Pixhawkをarmする  
        return: 成否  
### /sensor
#### ・LiDARHander()
    altitude() -> float  
      return: LiDARの値  
      
    update_altitude(altitude: float) -> None  
      altitudeを更新する  
      
    async invoke() -> None  
      LiDARのループを起動する  
### /flight
#### ・FlightController()  
### /control
#### ・PositionManager()  
#### ・Coordinates()  
    x() -> float  
      return: x  
      
    y() -> float  
      return: y  
      
    z() -> float  
      return: z  
      
    get() -> dict[str, float]  
      return: { "x": float, "y": float, "z": float }
      
    set_x(x: float) -> None  
      xの値を設定  
      
    set_y(x: float) -> None  
      yの値を設定  
      
    set_z(x: float) -> None  
      zの値を設定  
      
    set(x: float, y: float, z: float) -> None  
      x, y, zの値を設定  
### /config
#### ・ConfigManager(path: str)  
読み込む .ini ファイルのパスを指定  

    read(section: str, item: str) -> str  
      return: sectionで指定したセクションのitemの値  

    read_default(item: str) -> str  
      return: defaultセクションのitemの値  
      
    get_sections() -> list[str]  
      return: コンフィグファイルのセクション一覧  
      
    get_items(section: str) -> list[str]  
      return: sectionで指定したセクションのアイテム一覧
        
