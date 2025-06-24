class Vector3d:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self._x = x
        self._y = y
        self._z = z

    def get_x(self) -> float:
        return self._x

    def get_y(self) -> float:
        return self._y

    def get_z(self) -> float:
        return self._z

    def get(self) -> dict[str, float]:
        return {"x": self._x, "y": self._y, "z": self._z}

    def set_x(self, x: float):
        self._x = x

    def set_y(self, y: float):
        self._y = y

    def set_z(self, z: float):
        self._z = z

    def set(self, x: float, y: float, z: float):
        self._x = x
        self._y = y
        self._z = z
