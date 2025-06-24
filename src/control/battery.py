import mavsdk
import asyncio

class Battery_watch:
    def __init__(self,drone):
        self.drone = drone
        self.voltage: float = 0
        self.current_battery: float = 0
        self.remaining: float = 0
        self.temperature: float = 0
        

    def battery_info_update(self,info) -> None:
        self.voltage = info.voltage_v
        self.current_battery = info.current_battery_a
        self.remaining = info.remaining_percent
        self.temperature = info.temperature_degc
#################################################以下がオープン
    async def invoke_loop(self) -> None:
        async for info in self.drone.telemetry.battery():
            self.battery_info_update(info)
            print('updated')
            await asyncio.sleep(1)

    def voltage_v(self) -> float:
        return self.voltage
    
    def current_battery_a(self) -> float:
        return self.current_battery
    
    def remaining_percent(self) -> float:
        return self.remaining
    
    def temperature_degc(self) -> float:
        return self.temperature