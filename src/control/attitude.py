class Attitude: 
    def __init__(self):
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        self.qw: float = 1.0
        self.qx: float = 0.0
        self.qy: float = 0.0
        self.qz: float = 0.0

    def get_roll(self) -> float:
        return self.roll

    def get_pitch(self) -> float:
        return self.pitch

    def get_yaw(self) -> float:
        return self.yaw

    def get_quaternion(self) -> dict[str,float]:
        return {"qw": self.qw, "qx": self.qx, "qy": self.qy, "qz": self.qz}


    def get_attitude(self) -> dict[str,float,dict]:
        return {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw, "quaternion": self.get_quaternion()}

    def set_roll(self, roll: float):
        self.roll = roll
        return

    def set_pitch(self, pitch: float):
        self.pitch = pitch
        return

    def set_yaw(self, yaw: float):
        self.yaw = yaw
        return

    def set_quaternion(self, qw: float, qx: float, qy: float, qz: float):
        self.qw = qw
        self.qx = qx
        self.qy = qy
        self.qz = qz
