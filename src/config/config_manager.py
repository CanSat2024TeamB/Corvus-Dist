import configparser
import os
from pathlib import Path

class ConfigManager:
    default_path = Path(__file__).parent.parent.parent.joinpath("assets/config/config.ini")

    def __init__(self, path: str = str(default_path)):
        self.config_ini = configparser.ConfigParser()
        self.path = path
        self.load(path)
        return

    def load(self, path: str) -> None:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"{path} does not exist!")
        self.config_ini.read(path, encoding = "utf-8")
        return

    def save(self) -> None:
        with open(self.path, "w") as file:
            self.config_ini.write(file)

    def read(self, section: str, item: str) -> str:
        return self.config_ini.get(section, item)
    
    def read_int(self, section: str, item: str) -> int:
        return self.config_ini.getint(section, item)
    
    def read_float(self, section: str, item: str) -> int:
        return self.config_ini.getfloat(section, item)
    
    def read_bool(self, section: str, item: str) -> int:
        return self.config_ini.getboolean(section, item)
    
    def read_default(self, item: str) -> str:
        return self.config_ini.get("DEFAULT", item)
    
    def get_sections(self) -> list[str]:
        return self.config_ini.sections()

    def get_items(self, section: str) -> dict[str, str]:
        items = self.config_ini.items(section)
        return dict(items)
    
    def write(self, section: str, item: str, string: str) -> bool:
        if not self.config_ini.has_option(section, item):
            return False
        self.config_ini[section][item] = string
        self.save()