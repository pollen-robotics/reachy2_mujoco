# Had to copy this file from reachy2_sdk_server to get rid of rclpy dependency

from collections import defaultdict
from typing import List, Optional
from reachy2_sdk_api.component_pb2 import ComponentId


# Should have at least "name", "id" and "type" keys.
class Component:
    def __init__(
        self, name: str, id: int, type: str, extra: dict, state: dict = {}
    ) -> None:
        self.name = name
        self.id = id
        self.type = type
        self.extra = extra
        self.state = state

    def __repr__(self) -> str:
        return f"Component(name={self.name}, id={self.id}, type={self.type}, extra={self.extra} state={self.state})"

    def update_state(self, new_state) -> None:
        self.state.update(new_state)

    def update_command(self, new_cmd) -> None:
        if "position" in new_cmd:
            new_cmd["target_position"] = new_cmd.pop("position")

        self.state.update(new_cmd)

    def get_all_joints(self) -> List[str]:
        if self.type == "orbita2d":
            return [
                f"{self.name}_{self.extra['axis1']}",
                f"{self.name}_{self.extra['axis2']}",
            ]
        elif self.type == "orbita3d":
            return [
                f"{self.name}_{self.extra['axis1']}",
                f"{self.name}_{self.extra['axis2']}",
                f"{self.name}_{self.extra['axis3']}",
            ]
        else:
            raise ValueError(f"Unknown component type '{self.type}'.")


class ComponentsHolder:
    def __init__(self, config: dict) -> None:
        self.components = []
        self.by_name = {}
        self.by_id = {}
        self.by_type = defaultdict(list)

        self.config = config

    def add_component(
        self,
        name: str,
        # node_delegate: Optional[rclpy.node.Node] = None,
    ) -> None:
        # TODO
        # state = get_component_full_state(name, node_delegate)
        state = {"uid": 0}

        # We shift the id by one to avoid having an id of 0.
        id = int(state.pop("uid")) + 1

        c = Component(
            name=name,
            id=id,
            type=guess_component_type(name, self.config),
            extra=add_extra_if_any(name, self.config),
            state=state,
        )

        self.components.append(c)
        self.by_name[c.name] = c
        self.by_id[c.id] = c
        self.by_type[c.type].append(c)

        return c

    def get_by_component_id(self, component_id: ComponentId) -> Optional[Component]:
        if component_id.id:
            return self.get_by_id(component_id.id)
        elif component_id.name:
            return self.get_by_name(component_id.name)

    def get_by_type(self, component_type: str) -> List[Component]:
        if component_type in self.by_type:
            return self.by_type[component_type]

    def get_by_name(self, component_name: str) -> Optional[Component]:
        if component_name in self.by_name:
            return self.by_name[component_name]

    def get_by_id(self, component_id: int) -> Optional[Component]:
        if component_id in self.by_id:
            return self.by_id[component_id]


def guess_component_type(name: str, config: dict) -> str:
    for part in config["reachy"].values():
        for actuator in part.values():
            if actuator["name"] == name:
                return actuator["actuator"]

    return "unknown"


def add_extra_if_any(name: str, config: dict) -> dict:
    for part in config["reachy"].values():
        for actuator in part.values():
            if actuator["name"] == name:
                return actuator

    return {}
