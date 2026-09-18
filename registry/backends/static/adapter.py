"""Spawn-only backend: a vehicle in the world with no autopilot behind it."""

from __future__ import annotations

from typing import Any

from uavsim.adapter import AutopilotAdapter, Instance, ProcessSpec


class StaticAdapter(AutopilotAdapter):
    id = "static"

    def processes(self, inst: Instance) -> list[ProcessSpec]:
        return []

    def endpoints(self, inst: Instance) -> dict[str, Any]:
        return {"protocol": "none"}
