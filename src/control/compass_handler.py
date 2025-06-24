import asyncio
import mavsdk 
from control.attitude import Attitude

class CompassHandler:
    def __init__(self, drone):
        self.drone = drone
        self.attitude = Attitude()

    async def update_attitude(self, euler, quaternion) -> None:
        """Update the attitude with the latest sensor data"""
        self.attitude.set_roll(euler.roll_deg)
        self.attitude.set_pitch(euler.pitch_deg)
        self.attitude.set_yaw(euler.yaw_deg)
        self.attitude.set_quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        return
 #############################################################以下がオープン

    async def invoke_loop(self) -> None:
        while True:
            attitude_euler = self.drone.telemetry.attitude_euler()
            attitude_quaternion = self.drone.telemetry.attitude_quaternion()
            
            euler = await attitude_euler.__anext__()
            quaternion = await attitude_quaternion.__anext__()
            await self.update_attitude(euler, quaternion)
            await asyncio.sleep(0.05)


    def compass_attitude(self) -> Attitude:
        return self.attitude
    

    