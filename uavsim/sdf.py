"""Renders one instance model from airframe + binding + payloads.

Nothing is written to ``models/``: the SDF is a string that goes straight to the
entity-creation service.  Per-instance model directories on disk would drift
from the templates that generated them, and would have to exist before a run
could start.
"""

from __future__ import annotations

import xml.dom.minidom
from pathlib import Path
from typing import Any

import jinja2

from .adapter import Instance


class RenderError(Exception):
    pass


def _env(search_paths: list[Path]) -> jinja2.Environment:
    return jinja2.Environment(
        loader=jinja2.FileSystemLoader([str(p) for p in search_paths]),
        undefined=jinja2.StrictUndefined,
        trim_blocks=True,
        lstrip_blocks=False,
        keep_trailing_newline=True,
    )


def _render_file(path: Path, context: dict[str, Any]) -> str:
    env = _env([path.parent])
    try:
        return env.get_template(path.name).render(**context)
    except jinja2.TemplateError as exc:
        raise RenderError(f"{path}: {exc}") from exc


def render_model(inst: Instance, *, world_name: str, backend_vars: dict[str, Any]) -> str | None:
    """Return the instance SDF, or None when the backend spawns a plain uri."""
    vehicle = inst.vehicle
    binding = vehicle.binding

    if binding.overlay is None:
        return None

    base: dict[str, Any] = {
        "name": inst.name,
        "world": world_name,
        "index": inst.index,
        "backend_index": inst.backend_index,
        "sysid": inst.sysid,
        "airframe": vehicle.airframe,
        "vars": vehicle.binding_vars,
        **backend_vars,
    }

    controls: list[str] = []
    fragments: list[str] = []
    for rp in vehicle.payloads:
        mount = vehicle.airframe.mounts.get(rp.mount) if rp.mount else None
        ctx = {**base, "payload": rp, "mount": mount}
        frag_path = rp.spec.backend_fragments.get(vehicle.backend.id)
        if frag_path is not None:
            controls.append(_render_file(frag_path, ctx).rstrip("\n"))
        if rp.spec.fragment is not None:
            if rp.mount and mount is None:
                raise RenderError(
                    f"{inst.name}: payload {rp.id!r} needs mount {rp.mount!r}, "
                    f"which airframe {vehicle.airframe.id!r} does not define")
            fragments.append(_render_file(rp.spec.fragment, ctx).rstrip("\n"))

    body = _render_file(binding.overlay,
                        {**base, "payload_controls": "\n".join(controls)}).rstrip("\n")

    sdf = "\n".join([
        "<?xml version='1.0'?>",
        '<sdf version="1.9">',
        f'  <model name="{inst.name}">',
        body,
        *fragments,
        "  </model>",
        "</sdf>",
        "",
    ])
    _check_xml(sdf, inst.name)
    return sdf


def _check_xml(text: str, name: str) -> None:
    try:
        xml.dom.minidom.parseString(text)
    except Exception as exc:                        # noqa: BLE001 - surfaces as RenderError
        raise RenderError(f"{name}: rendered SDF is not well-formed XML: {exc}") from exc


def render_string(template: str, context: dict[str, Any]) -> str:
    """Render an inline template (used for model uris and world fragments)."""
    env = jinja2.Environment(undefined=jinja2.StrictUndefined,
                             trim_blocks=True, keep_trailing_newline=True)
    try:
        return env.from_string(template).render(**context)
    except jinja2.TemplateError as exc:
        raise RenderError(f"{template!r}: {exc}") from exc
