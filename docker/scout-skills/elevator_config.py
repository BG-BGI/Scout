"""Per-site elevator topology: sites/<name>/elevator.json (ADR-0030).

Schema v1 (hand-authored, optional file — no fleet-status scaffolding):

{
  "version": 1,
  "default_elevator": "main",
  "elevators": {
    "main": {
      "equipment_number": "EQ-1-1-1",
      "identity": {"type": "email", "textId": "robot@example.com"},   # optional
      "floors": {
        "2": {"label": "0", "door_waypoint": "elev_main_lobby",
               "entrance_side": "Front", "board_depth_m": 1.4,
               "exit": "reverse", "exit_move_m": 1.7}
      }
    }
  }
}

Floor keys are the API's floorNumber — the 1-based index among SERVED stops,
NOT the displayed label ("label" is documentation, cross-check GET /floors).
Deployment identity/gateway (base URL, certs, bearer) is env, not this file.
Opened per tool call so a site switch applies live (ADR-0023).
"""

from __future__ import annotations

ELEVATOR_CONFIG_VERSION = 1

ENTRANCE_SIDES = {"None", "Front", "Rear", "Left", "Right"}
EXIT_DIRECTIONS = {"reverse", "forward"}

DEFAULT_BOARD_DEPTH_M = 1.4
DEFAULT_EXIT_EXTRA_M = 0.3  # exit_move_m default = board_depth_m + this


def load_elevator_config(data: object) -> dict:
    """Validate + normalize elevator.json. Raises ValueError naming the bad
    key. Returns {"default_elevator": str|None, "elevators": {name: {...}}}
    with floor keys as int and per-floor defaults filled in."""
    if not isinstance(data, dict):
        raise ValueError("elevator.json must be a JSON object")
    version = data.get("version")
    if version != ELEVATOR_CONFIG_VERSION:
        raise ValueError(
            f"elevator.json version {version!r} unsupported (expected {ELEVATOR_CONFIG_VERSION})"
        )
    elevators_raw = data.get("elevators")
    if not isinstance(elevators_raw, dict) or not elevators_raw:
        raise ValueError("elevator.json needs a non-empty 'elevators' object")

    elevators: dict[str, dict] = {}
    for name, entry in elevators_raw.items():
        if not isinstance(entry, dict):
            raise ValueError(f"elevators[{name!r}] must be an object")
        equipment = entry.get("equipment_number")
        if not equipment or not isinstance(equipment, str):
            raise ValueError(f"elevators[{name!r}].equipment_number is required (string)")
        identity = entry.get("identity")
        if identity is not None and (
            not isinstance(identity, dict) or "type" not in identity
        ):
            raise ValueError(f"elevators[{name!r}].identity must be an object with 'type'")
        floors_raw = entry.get("floors") or {}
        if not isinstance(floors_raw, dict):
            raise ValueError(f"elevators[{name!r}].floors must be an object")
        floors: dict[int, dict] = {}
        for key, floor in floors_raw.items():
            try:
                num = int(key)
            except (TypeError, ValueError):
                raise ValueError(
                    f"elevators[{name!r}].floors key {key!r} is not an integer floorNumber"
                ) from None
            if num < 1:
                raise ValueError(
                    f"elevators[{name!r}].floors[{key!r}]: floorNumber must be >= 1 "
                    "(served-stop index, not the label)"
                )
            if not isinstance(floor, dict):
                raise ValueError(f"elevators[{name!r}].floors[{key!r}] must be an object")
            side = floor.get("entrance_side", "Front")
            if side not in ENTRANCE_SIDES:
                raise ValueError(
                    f"elevators[{name!r}].floors[{key!r}].entrance_side {side!r} "
                    f"not one of {sorted(ENTRANCE_SIDES)}"
                )
            exit_dir = floor.get("exit", "reverse")
            if exit_dir not in EXIT_DIRECTIONS:
                raise ValueError(
                    f"elevators[{name!r}].floors[{key!r}].exit {exit_dir!r} "
                    f"not one of {sorted(EXIT_DIRECTIONS)}"
                )
            depth = float(floor.get("board_depth_m", DEFAULT_BOARD_DEPTH_M))
            floors[num] = {
                "label": floor.get("label"),
                "door_waypoint": floor.get("door_waypoint"),
                "entrance_side": side,
                "board_depth_m": depth,
                "exit": exit_dir,
                "exit_move_m": float(floor.get("exit_move_m", depth + DEFAULT_EXIT_EXTRA_M)),
            }
        elevators[name] = {
            "equipment_number": equipment,
            "identity": identity,
            "floors": floors,
        }

    default = data.get("default_elevator")
    if default is not None and default not in elevators:
        raise ValueError(f"default_elevator {default!r} not in elevators {sorted(elevators)}")
    if default is None and len(elevators) == 1:
        default = next(iter(elevators))
    return {"default_elevator": default, "elevators": elevators}


def resolve_elevator(cfg: dict, name: str | None) -> tuple[str, dict]:
    """Pick an elevator by name or the default. Raises ValueError."""
    if name is None:
        name = cfg.get("default_elevator")
        if name is None:
            raise ValueError(
                f"no elevator named and no default_elevator — have: {sorted(cfg['elevators'])}"
            )
    entry = cfg["elevators"].get(name)
    if entry is None:
        raise ValueError(f"no elevator {name!r} — have: {sorted(cfg['elevators'])}")
    return name, entry
