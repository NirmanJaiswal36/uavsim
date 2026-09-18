"""Geometry of the ground control station site.

One source of truth: ``world.py`` builds the ``gcs_site`` SDF from these numbers
and ``allocator.py`` places vehicles on them, so pads in the picture and pads in
the allocation can never drift apart.

Site frame (before ``gcs_pose`` is applied): +X points at the ROI, the apron is
behind the runway threshold, the shelter sits to the south.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .adapter import Pose


@dataclass(frozen=True)
class SiteLayout:
    # landing pads for anything that takes off vertically
    pad_rows: int = 4
    pad_cols: int = 6
    pad_spacing_m: float = 6.0
    pad_radius_m: float = 1.6
    apron_x_m: float = -40.0      # apron centre, behind the runway threshold
    apron_y_m: float = 0.0

    # runway for anything that rolls
    runway_length_m: float = 120.0
    runway_width_m: float = 10.0
    runway_x0_m: float = -10.0    # threshold; runway runs +X, toward the ROI
    runway_slot_spacing_m: float = 12.0
    runway_slot_count: int = 6

    def pad_slots(self) -> list[Pose]:
        """Pad centres, nearest-the-runway first, left to right."""
        slots: list[Pose] = []
        for row in range(self.pad_rows):
            for col in range(self.pad_cols):
                x = self.apron_x_m + (self.pad_rows - 1 - row) * self.pad_spacing_m \
                    - (self.pad_rows - 1) * self.pad_spacing_m / 2.0
                y = self.apron_y_m + (col - (self.pad_cols - 1) / 2.0) * self.pad_spacing_m
                slots.append(Pose(x=x, y=y))
        return slots

    def runway_slots(self) -> list[Pose]:
        """Take-off positions along the runway centreline, threshold first."""
        return [Pose(x=self.runway_x0_m + 6.0 + i * self.runway_slot_spacing_m, y=0.0)
                for i in range(self.runway_slot_count)]

    @property
    def pad_capacity(self) -> int:
        return self.pad_rows * self.pad_cols

    def min_pad_spacing_ok(self, footprint_radius_m: float) -> bool:
        return self.pad_spacing_m >= 2.0 * footprint_radius_m


DEFAULT_SITE = SiteLayout()
DEFAULT_GCS_POSE = Pose(x=-500.0, y=0.0, z=0.0)


def compose(site_pose: Pose, local: Pose) -> Pose:
    """Local site pose -> world pose (yaw-only rotation, which is all we use)."""
    cos_y, sin_y = math.cos(site_pose.yaw), math.sin(site_pose.yaw)
    return Pose(
        x=site_pose.x + local.x * cos_y - local.y * sin_y,
        y=site_pose.y + local.x * sin_y + local.y * cos_y,
        z=site_pose.z + local.z,
        roll=local.roll,
        pitch=local.pitch,
        yaw=site_pose.yaw + local.yaw,
    )


def heading_to(origin: Pose, target_x: float = 0.0, target_y: float = 0.0) -> float:
    """Yaw (rad) from ``origin`` toward a world point -- used to face the ROI."""
    return math.atan2(target_y - origin.y, target_x - origin.x)
