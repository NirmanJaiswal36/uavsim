"""Loads and validates the vehicle registry.

The registry is data, not code: everything under ``registry/`` is YAML plus
Jinja templates, validated against ``registry/schemas/*.json``.  The only code
it may carry is a backend adapter (``registry/backends/<id>/adapter.py``), which
is imported by path so that adding a backend needs no change to the core.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

import jsonschema
import yaml

from .spec import (Airframe, Backend, Binding, Payload, ResolvedPayload,
                   ResolvedVehicle, VehicleType)

PKG_ROOT = Path(__file__).resolve().parent.parent
REGISTRY_DIR = PKG_ROOT / "registry"


class RegistryError(Exception):
    """Bad or missing registry data. Always names the offending file."""


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise RegistryError(f"{path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RegistryError(f"{path}: expected a YAML mapping")
    return data


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursive dict merge; ``override`` wins, lists are replaced not appended."""
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


class Registry:
    def __init__(self, root: Path | None = None):
        self.root = Path(root or REGISTRY_DIR)
        self._schemas: dict[str, dict] = {}
        self.airframes: dict[str, Airframe] = {}
        self.backends: dict[str, Backend] = {}
        self.payloads: dict[str, Payload] = {}
        self.vehicles: dict[str, VehicleType] = {}
        self._load()

    # -- schemas -------------------------------------------------------------

    def schema(self, name: str) -> dict:
        if name not in self._schemas:
            path = self.root / "schemas" / f"{name}.schema.json"
            if not path.exists():
                raise RegistryError(f"missing schema {path}")
            self._schemas[name] = json.loads(path.read_text())
        return self._schemas[name]

    def validate(self, data: dict, schema: str, where: Path | str) -> None:
        try:
            jsonschema.validate(data, self.schema(schema))
        except jsonschema.ValidationError as exc:
            loc = "/".join(str(p) for p in exc.absolute_path) or "<root>"
            raise RegistryError(f"{where}: {schema} invalid at {loc}: {exc.message}") from None

    # -- loading -------------------------------------------------------------

    def _load(self) -> None:
        for path in sorted(self.root.glob("airframes/*/airframe.yaml")):
            af = self._airframe(path)
            self.airframes[af.id] = af
        for path in sorted(self.root.glob("backends/*/backend.yaml")):
            be = self._backend(path)
            self.backends[be.id] = be
        for path in sorted(self.root.glob("payloads/*/payload.yaml")):
            pl = self._payload(path)
            self.payloads[pl.id] = pl
        for path in sorted(self.root.glob("vehicles/*.yaml")):
            vt = self._vehicle(path)
            self.vehicles[vt.id] = vt

    def _rel(self, base: Path, value: str, where: Path) -> Path:
        path = (base / value).resolve()
        if not path.exists():
            raise RegistryError(f"{where}: references missing file {path}")
        return path

    def _airframe(self, path: Path) -> Airframe:
        data = _load_yaml(path)
        self.validate(data, "airframe", path)
        if data["id"] != path.parent.name:
            raise RegistryError(f"{path}: id {data['id']!r} != directory {path.parent.name!r}")
        bindings = {}
        for backend_id, raw in (data.get("bindings") or {}).items():
            if bool(raw.get("overlay")) == bool(raw.get("model_uri")):
                raise RegistryError(
                    f"{path}: binding {backend_id!r} needs exactly one of overlay / model_uri")
            bindings[backend_id] = Binding(
                backend=backend_id,
                overlay=self._rel(path.parent, raw["overlay"], path) if raw.get("overlay") else None,
                model_uri=raw.get("model_uri"),
                vars=raw.get("vars") or {},
            )
        return Airframe(
            id=data["id"], cls=data["class"], launch=data["launch"], dir=path.parent,
            model_uri=data.get("model_uri"), base_link=data.get("base_link"),
            imu_sensor=data.get("imu_sensor"), geometry=data.get("geometry") or {},
            performance=data.get("performance") or {}, mounts=data.get("mounts") or {},
            bindings=bindings, description=data.get("description", ""),
        )

    def _backend(self, path: Path) -> Backend:
        data = _load_yaml(path)
        self.validate(data, "backend", path)
        if data["id"] != path.parent.name:
            raise RegistryError(f"{path}: id {data['id']!r} != directory {path.parent.name!r}")
        module, _, cls = data["adapter"].partition(":")
        self._rel(path.parent, f"{module}.py", path)
        return Backend(
            id=data["id"], dir=path.parent, adapter=data["adapter"],
            requires=data.get("requires") or {}, settings=data.get("settings") or {},
            capabilities=data.get("capabilities") or {}, description=data.get("description", ""),
        )

    def _payload(self, path: Path) -> Payload:
        data = _load_yaml(path)
        self.validate(data, "payload", path)
        if data["id"] != path.parent.name:
            raise RegistryError(f"{path}: id {data['id']!r} != directory {path.parent.name!r}")
        return Payload(
            id=data["id"], dir=path.parent,
            fragment=self._rel(path.parent, data["fragment"], path) if data.get("fragment") else None,
            mount=data.get("mount"), params=data.get("params") or {},
            backend_fragments={
                b: self._rel(path.parent, f, path)
                for b, f in (data.get("backend_fragments") or {}).items()
            },
            provides=data.get("provides") or [], singleton=bool(data.get("singleton", False)),
            description=data.get("description", ""),
        )

    def _vehicle(self, path: Path) -> VehicleType:
        data = _load_yaml(path)
        self.validate(data, "vehicle", path)
        if data["id"] != path.stem:
            raise RegistryError(f"{path}: id {data['id']!r} != file name {path.stem!r}")
        return VehicleType(
            id=data["id"], airframe=data["airframe"], backend=data["backend"],
            payloads=data.get("payloads") or [], backend_config=data.get("backend_config") or {},
            binding_vars=data.get("binding_vars") or {}, description=data.get("description", ""),
        )

    # -- resolution ----------------------------------------------------------

    def resolve(self, vehicle_id: str, *,
                backend_config: dict[str, Any] | None = None,
                payload_params: dict[str, Any] | None = None) -> ResolvedVehicle:
        """Merge a vehicle type into the flat view everything else consumes."""
        if vehicle_id not in self.vehicles:
            raise RegistryError(
                f"unknown vehicle type {vehicle_id!r}; known: {', '.join(sorted(self.vehicles))}")
        vt = self.vehicles[vehicle_id]

        if vt.airframe not in self.airframes:
            raise RegistryError(f"vehicle {vt.id!r}: unknown airframe {vt.airframe!r}")
        if vt.backend not in self.backends:
            raise RegistryError(f"vehicle {vt.id!r}: unknown backend {vt.backend!r}")
        airframe, backend = self.airframes[vt.airframe], self.backends[vt.backend]

        if backend.id not in airframe.bindings:
            raise RegistryError(
                f"vehicle {vt.id!r}: airframe {airframe.id!r} has no {backend.id!r} binding "
                f"(has: {', '.join(sorted(airframe.bindings)) or 'none'})")
        binding = airframe.bindings[backend.id]

        payloads = []
        seen: set[str] = set()
        for entry in vt.payloads:
            if entry["type"] not in self.payloads:
                raise RegistryError(f"vehicle {vt.id!r}: unknown payload {entry['type']!r}")
            spec = self.payloads[entry["type"]]
            pid = entry.get("id") or spec.id
            if pid in seen:
                raise RegistryError(f"vehicle {vt.id!r}: duplicate payload id {pid!r}")
            seen.add(pid)
            mount = entry.get("mount") or spec.mount
            if mount and mount not in airframe.mounts:
                raise RegistryError(
                    f"vehicle {vt.id!r}: payload {pid!r} wants mount {mount!r}, "
                    f"airframe {airframe.id!r} offers {', '.join(sorted(airframe.mounts)) or 'none'}")
            params = _merge(spec.params, entry.get("params") or {})
            params = _merge(params, (payload_params or {}).get(pid, {}))
            payloads.append(ResolvedPayload(id=pid, spec=spec, mount=mount, params=params))

        return ResolvedVehicle(
            id=vt.id, airframe=airframe, backend=backend, binding=binding, payloads=payloads,
            backend_config=_merge(vt.backend_config, backend_config or {}),
            binding_vars=_merge(binding.vars, vt.binding_vars),
        )

    # -- adapters ------------------------------------------------------------

    def adapter_class(self, backend_id: str):
        """Import ``registry/backends/<id>/<module>.py`` and return its adapter class."""
        if backend_id not in self.backends:
            raise RegistryError(f"unknown backend {backend_id!r}")
        backend = self.backends[backend_id]
        module_name, _, cls_name = backend.adapter.partition(":")
        mod_path = backend.dir / f"{module_name}.py"
        qualified = f"uavsim_backend_{backend_id}_{module_name}"
        if qualified in sys.modules:
            module = sys.modules[qualified]
        else:
            spec = importlib.util.spec_from_file_location(qualified, mod_path)
            if spec is None or spec.loader is None:
                raise RegistryError(f"cannot import adapter {mod_path}")
            module = importlib.util.module_from_spec(spec)
            sys.modules[qualified] = module
            spec.loader.exec_module(module)
        try:
            return getattr(module, cls_name)
        except AttributeError:
            raise RegistryError(f"{mod_path}: no class {cls_name!r}") from None
