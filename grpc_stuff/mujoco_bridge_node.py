from components import ComponentsHolder
from parts import PartsHolder
from utils import parse_reachy_config


class MujocoBridgeNode:
    def __init__(self, config):
        self.config = parse_reachy_config(config)
        self.components = ComponentsHolder(self.config)
        self.parts = PartsHolder(self.config, self.components)


if __name__ == "__main__":
    mbn = MujocoBridgeNode("reachy.yaml")
