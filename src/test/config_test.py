import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from config.config_manager import ConfigManager

config = ConfigManager()
section = "ARLISS"
speed = config.read_float(section, "Speed")
hov_alt = config.read_float(section, "HovAlt")
target_lon = config.read_float(section, "TargetLon")
target_lat = config.read_float(section, "TargetLat")
goal_radius = config.read_float(section, "GoalRadius")
offboard_acceptable_distance = config.read_float(section, "OffboardAcceptableDistance")

lora_sync = config.read_int(section, "LoraSync")
lora_freq = config.read_int(section, "LoraFreq")
lora_sf = config.read_int(section, "LoraSf")
lora_bw = config.read_int(section, "LoraBw")
lora_pwr = config.read_int(section, "LoraPwr")

print(speed, type(speed))
print(hov_alt, type(hov_alt))
print(target_lon, type(target_lon))
print(target_lat, type(target_lat))
print(goal_radius, type(goal_radius))
print(offboard_acceptable_distance, type(offboard_acceptable_distance))

print(lora_sync, type(lora_sync))
print(lora_freq, type(lora_freq))
print(lora_sf, type(lora_sf))
print(lora_bw, type(lora_bw))
print(lora_pwr, type(lora_pwr))

a = []
config_section = "ARLISS"
a.append(config.read_int(config_section, "JudgeStorageBorderLight"))
a.append(config.read_int(config_section, "JudgeStorageCountmax"))
a.append(config.read_int(config_section, "JudgeStorageMaxtime"))
a.append(config.read_float(config_section, "JudgeStorageSleepTime"))

#放出判定用定数 
a.append(config.read_int(config_section, "JudgeReleaseMaxtime"))
a.append(config.read_int(config_section, "JudgeReleaseLigCountmax"))
a.append(config.read_int(config_section, "JudgeReleasePreCountmax"))
a.append(config.read_int(config_section, "JudgeReleaseBorderLight"))
a.append(config.read_float(config_section, "JudgeReleaseSleepTime"))
a.append(config.read_int(config_section, "StableJudgeCountRelease"))

#着地判定用定数
a.append(config.read_int(config_section, "JudgeLandingMaxtime"))
a.append(config.read_int(config_section, "StableJudgeCountVel"))
a.append(config.read_int(config_section, "StableJudgeCountLand"))
a.append(config.read_int(config_section, "StableJudgeMaxFailure"))

for item in a:
    print(item, type(item))