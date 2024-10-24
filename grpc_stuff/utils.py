import yaml


def parse_reachy_config(reachy_config_path: str) -> dict:
    with open(reachy_config_path, "r") as f:
        config = yaml.safe_load(f)
    return config
