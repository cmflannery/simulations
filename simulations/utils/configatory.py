"""configatory.py provides a simple tool set for parsing config files"""

import configparser


class Vehicle():
    def __init__(self, config_file):
        self.config = configparser.ConfigParser()
        self.config.read(config_file)

    def process_config(self):
        pass
