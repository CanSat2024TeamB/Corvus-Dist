from control.vector3d import Vector3d


class Coordinates(Vector3d):
    def __init__(self, longitude: float = 0.0, latitude: float = 0.0, altitude: float = 0.0):
        super().__init__(longitude, latitude, altitude)

    def longitude(self) -> float:
        return self.get_x()

    def latitude(self) -> float:
        return self.get_y()

    def altitude(self) -> float:
        return self.get_z()

    def set_longitude(self, longitude: float) -> None:
        self.set_x(longitude)

    def set_latitude(self, latitude: float) -> None:
        self.set_y(latitude)

    def set_altitude(self, altitude: float) -> None:
        self.set_z(altitude)