"""Geometry of the ground control station site.

One source of truth: ``world.py`` builds the ``gcs_site`` SDF from these numbers
and ``allocator.py`` places vehicles on them, so pads in the picture and pads in
the allocation can never drift apart.

Site frame (before ``gcs_pose`` is applied): +X points at the ROI, the apron is
centred on the site origin, the shelter sits to the south.  Every vehicle is a
multirotor and takes off from a pad, so there is no runway.

Where ``DEFAULT_GCS_POSE`` comes from.  The site is not a point: the apron's front
edge is the forest-facing extremity of the whole GCS, and the standoff we care
about is the open ground between that edge and the forest.  We want 100 m of it::

    forest_xmin - (X + apron_front_x) = 100
    (-250)      - (X + 15)            = 100   ->   X = -365.0

which lays the site out like this (world x, forest centre at 0)::

    apron rear       -380.0
    pad rows         -374.0 .. -356.0     the multirotors spawn here
    apron front      -350.0
    ................ 100 m of open ground
    forest edge      -250.0               the boundary posts
    forest centre       0.0               also the geographic origin

So ``distance_to_roi_m`` in the manifest reads 365 m, not 100: it is measured from
the site origin to the ROI centre, which is a different quantity.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .adapter import Pose


@dataclass(frozen=True)
class SiteLayout:
    pad_rows: int = 4
    pad_cols: int = 6
    pad_spacing_m: float = 6.0
    pad_radius_m: float = 1.6
    apron_x_m: float = 0.0        # apron centre
    apron_y_m: float = 0.0
    apron_margin_m: float = 3.0   # apron extends this far past the pad grid

    def pad_slots(self) -> list[Pose]:
        """Pad centres, nearest-the-ROI row first, left to right."""
        slots: list[Pose] = []
        for row in range(self.pad_rows):
            for col in range(self.pad_cols):
                x = self.apron_x_m + (self.pad_rows - 1 - row) * self.pad_spacing_m \
                    - (self.pad_rows - 1) * self.pad_spacing_m / 2.0
                y = self.apron_y_m + (col - (self.pad_cols - 1) / 2.0) * self.pad_spacing_m
                slots.append(Pose(x=x, y=y))
        return slots

    @property
    def apron_length_m(self) -> float:
        """Apron size along +X, toward the ROI."""
        return self.pad_rows * self.pad_spacing_m + 2 * self.apron_margin_m

    @property
    def apron_width_m(self) -> float:
        return self.pad_cols * self.pad_spacing_m + 2 * self.apron_margin_m

    @property
    def front_x_m(self) -> float:
        """The forest-facing extremity of the site, in the site frame."""
        return self.apron_x_m + self.apron_length_m / 2.0

    @property
    def pad_capacity(self) -> int:
        return self.pad_rows * self.pad_cols

    def min_pad_spacing_ok(self, footprint_radius_m: float) -> bool:
        return self.pad_spacing_m >= 2.0 * footprint_radius_m


DEFAULT_SITE = SiteLayout()
DEFAULT_GCS_POSE = Pose(x=-365.0, y=0.0, z=0.0)   # 100 m of open ground; see above


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
