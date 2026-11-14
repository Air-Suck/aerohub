"""Apply common PX4 connections to an upstream-resolved scene dictionary."""
from copy import deepcopy


def configure_scene(scene: dict, manifest: dict) -> dict:
    scene = deepcopy(scene)
    actors = {actor["name"]: actor for actor in scene.get("actors", [])
              if actor.get("type") == "robot"}
    vehicles = manifest["vehicles"]
    if set(actors) != {vehicle["name"] for vehicle in vehicles}:
        raise ValueError("scene robots must match the manager vehicles exactly")
    for vehicle in vehicles:
        controller = actors[vehicle["name"]]["robot-config"].get("controller", {})
        if controller.get("type") != "px4-api":
            raise ValueError(f"{vehicle['name']}: requires upstream px4-api controller")
        px4 = vehicle["px4"]
        settings = controller.setdefault("px4-settings", {})
        settings.update({
            "use-tcp": True, "use-serial": False,
            "lock-step": px4.get("lockstep", True),
            "tcp-port": px4["tcp_port"],
            "control-port-local": px4["control_local_port"],
            "control-port-remote": px4["control_remote_port"],
        })
    return scene
