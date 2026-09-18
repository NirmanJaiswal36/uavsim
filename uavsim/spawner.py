"""Creates entities in a running Gazebo world.

Spawning at runtime (rather than listing vehicles in the world file) is what
lets one world serve any fleet, and it is also the only way to give each
instance its own ArduPilotPlugin port: SDFormat cannot override a plugin that
an <include>d model already declares, which is why the old setup needed a
generated model directory per drone.
"""

from __future__ import annotations

import math
import subprocess
import time
from dataclasses import dataclass

from .adapter import Pose, SpawnRequest


class SpawnError(Exception):
    pass


@dataclass
class Spawner:
    world: str
    timeout_ms: int = 10000

    def __post_init__(self) -> None:
        from gz.transport13 import Node          # imported lazily: needs the gz python bindings
        self._node = Node()

    # -- service calls -------------------------------------------------------

    def wait_for_world(self, timeout_s: float = 60.0) -> bool:
        """True once the server answers scene/info for this world."""
        deadline = time.time() + timeout_s
        service = f"/world/{self.world}/scene/info"
        while time.time() < deadline:
            try:
                out = subprocess.run(["gz", "service", "-l"], capture_output=True,
                                     text=True, timeout=10)
            except (OSError, subprocess.TimeoutExpired):
                return False
            if service in out.stdout:
                return True
            time.sleep(1.0)
        return False

    def spawn(self, request: SpawnRequest) -> None:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory

        if not request.sdf and not request.model_uri:
            raise SpawnError(f"{request.name}: spawn request has neither sdf nor model_uri")

        req = EntityFactory()
        req.name = request.name
        req.allow_renaming = False
        if request.sdf:
            req.sdf = request.sdf
        else:
            req.sdf_filename = request.model_uri
        req.pose.CopyFrom(_pose_msg(request.pose))

        ok, reply = self._node.request(f"/world/{self.world}/create", req,
                                       EntityFactory, Boolean, self.timeout_ms)
        if not ok:
            raise SpawnError(
                f"{request.name}: create service did not answer in {self.timeout_ms} ms "
                f"-- is 'gz sim' running world {self.world!r}?")
        if not reply.data:
            raise SpawnError(
                f"{request.name}: Gazebo refused the entity (duplicate name, bad SDF, or an "
                f"unresolvable model uri -- check GZ_SIM_RESOURCE_PATH)")

    def remove(self, name: str) -> bool:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity

        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        ok, reply = self._node.request(f"/world/{self.world}/remove", req,
                                       Entity, Boolean, self.timeout_ms)
        return bool(ok and reply.data)


def _pose_msg(pose: Pose):
    from gz.msgs10.pose_pb2 import Pose as PoseMsg

    msg = PoseMsg()
    msg.position.x, msg.position.y, msg.position.z = pose.x, pose.y, pose.z
    cr, sr = math.cos(pose.roll / 2), math.sin(pose.roll / 2)
    cp, sp = math.cos(pose.pitch / 2), math.sin(pose.pitch / 2)
    cy, sy = math.cos(pose.yaw / 2), math.sin(pose.yaw / 2)
    msg.orientation.w = cr * cp * cy + sr * sp * sy
    msg.orientation.x = sr * cp * cy - cr * sp * sy
    msg.orientation.y = cr * sp * cy + sr * cp * sy
    msg.orientation.z = cr * cp * sy - sr * sp * cy
    return msg
