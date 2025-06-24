
from control.attitude import Attitude
from control.coordinates import Coordinates
import numpy as np

class PositionManager:
    def __init__(self, drone, GPS_handler, Compass_handler, LiDARHandler):
        self.drone = drone
        self.gps_handler = GPS_handler
        self.compass_handler = Compass_handler
        self.lidar_handler = LiDARHandler

    
    def raw_altitude(self) -> float:
        return self.lidar_handler.get_altitude()
    
    def raw_coordinates(self) -> Coordinates:
        return self.gps_handler.gps_coordinates()
    
    def raw_attitude(self) -> Attitude:
        return self.lidar_handler.attitude()

    def adjusted_altitude(self) -> float:
        lidar = self.lidar_handler.get_altitude()
        Pitch_deg = self.compass_handler.compass_attitude().get_pitch()
        Roll_deg = self.compass_handler.compass_attitude().get_roll()
        adjusted_altitude = lidar * np.cos(np.deg2rad(Pitch_deg)) * np.cos(np.deg2rad(Roll_deg))
        return adjusted_altitude
    
    def adjusted_coordinates_lon(self) -> float:
        longitude = self.gps_handler.gps_coordinates().longitude()
        return longitude
    
    def adjusted_coordinates_lat(self) -> float:
        latitude = self.gps_handler.gps_coordinates().latitude()
        return latitude
    
    def adjusted_coordinates_AMSL(self) -> float:
        AMSL = self.gps_handler.gps_coordinates().altitude()
        return AMSL
    
    def yaw_deg(self) -> float:
        return self.compass_handler.compass_attitude().get_yaw()