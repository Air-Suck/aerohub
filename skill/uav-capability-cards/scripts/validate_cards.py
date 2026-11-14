#!/usr/bin/env python3
"""Validate AeroHub Capability Card Schema v1 and its canonical registries."""

import argparse
import glob
import json
import os
from pathlib import Path

import yaml


CARD_REQUIRED = {
    "schema_version", "id", "version", "type", "name", "description",
    "implementation", "inputs", "outputs", "interfaces", "lifecycle",
    "hardware_requirements", "resource_access",
}
CARD_ALLOWED = CARD_REQUIRED | {"demo"}


def load_yaml(path: Path):
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    json.dumps(value, ensure_ascii=False)
    return value


def exact_keys(errors, label, value, required, allowed=None):
    if not isinstance(value, dict):
        errors.append(f"{label}: expected mapping")
        return
    allowed = required if allowed is None else allowed
    missing = required - set(value)
    extra = set(value) - allowed
    if missing:
        errors.append(f"{label}: missing fields {sorted(missing)}")
    if extra:
        errors.append(f"{label}: unknown fields {sorted(extra)}")


def validate(repository: Path):
    errors = []
    cards_root = repository / "cards"
    capability_registry = load_yaml(cards_root / "capability-types.yaml")
    hardware_registry = load_yaml(cards_root / "hardware-types.yaml")

    exact_keys(errors, "capability registry", capability_registry, {"schema_version", "capability_types"})
    exact_keys(errors, "hardware registry", hardware_registry, {"schema_version", "hardware_types"})
    if capability_registry.get("schema_version") != "1.0":
        errors.append("capability registry: schema_version must be 1.0")
    if hardware_registry.get("schema_version") != "1.0":
        errors.append("hardware registry: schema_version must be 1.0")

    capability_types = capability_registry.get("capability_types", [])
    hardware_types = hardware_registry.get("hardware_types", [])
    capability_ids = [entry.get("id") for entry in capability_types]
    hardware_ids = [entry.get("id") for entry in hardware_types]
    if None in capability_ids or len(capability_ids) != len(set(capability_ids)):
        errors.append("capability registry: missing or duplicate IDs")
    if None in hardware_ids or len(hardware_ids) != len(set(hardware_ids)):
        errors.append("hardware registry: missing or duplicate IDs")

    for index, entry in enumerate(capability_types):
        label = f"capability_types[{index}]"
        exact_keys(errors, label, entry, {"id", "name", "description", "transport", "semantics"})
        exact_keys(errors, f"{label}.transport", entry.get("transport"), {"protocol", "schema", "qos"})
        exact_keys(
            errors, f"{label}.transport.qos", entry.get("transport", {}).get("qos"),
            {"reliability", "durability", "history", "depth"},
        )
        exact_keys(
            errors, f"{label}.semantics", entry.get("semantics"),
            {"unit_system", "units", "coordinate_frame", "axes", "timestamp"},
        )

    for index, entry in enumerate(hardware_types):
        exact_keys(errors, f"hardware_types[{index}]", entry, {"id", "name", "description"})

    card_paths = sorted(Path(path) for path in glob.glob(str(cards_root / "*" / "*.yaml")))
    cards = [(path, load_yaml(path)) for path in card_paths]
    card_ids = [card.get("id") for _, card in cards]
    if None in card_ids or len(card_ids) != len(set(card_ids)):
        errors.append("Cards: missing or duplicate IDs")

    for path, card in cards:
        label = str(path.relative_to(repository))
        exact_keys(errors, label, card, CARD_REQUIRED, CARD_ALLOWED)
        if card.get("schema_version") != "1.0":
            errors.append(f"{label}: schema_version must be 1.0")
        if card.get("type") not in {"capability", "tool"}:
            errors.append(f"{label}: invalid type {card.get('type')}")

        implementation = card.get("implementation", {})
        exact_keys(
            errors, f"{label}.implementation", implementation,
            {"component_id", "manifest", "shared_runtime", "coupled_capabilities"},
        )
        manifest_path = implementation.get("manifest")
        if manifest_path:
            resolved_manifest = repository / manifest_path
            if not resolved_manifest.is_file():
                errors.append(f"{label}: manifest does not exist: {manifest_path}")
            else:
                manifest = load_yaml(resolved_manifest)
                if manifest.get("id") != implementation.get("component_id"):
                    errors.append(f"{label}: component_id does not match manifest ID")
                provided = manifest.get("capabilities", manifest.get("provides", []))
                provided_ids = [item.get("id") if isinstance(item, dict) else item for item in provided]
                if card.get("id") not in provided_ids:
                    errors.append(f"{label}: Card ID is not listed by its component manifest")

        for coupled in implementation.get("coupled_capabilities", []):
            if coupled not in card_ids:
                errors.append(f"{label}: unknown coupled Card {coupled}")
                continue
            target = next(candidate for _, candidate in cards if candidate["id"] == coupled)
            if card["id"] not in target["implementation"]["coupled_capabilities"]:
                errors.append(f"{label}: coupled Card {coupled} does not link back")

        for direction in ("inputs", "outputs"):
            for index, item in enumerate(card.get(direction, [])):
                exact_keys(errors, f"{label}.{direction}[{index}]", item, {"id", "capability_type"})
                if item.get("capability_type") not in capability_ids:
                    errors.append(f"{label}: unknown capability type {item.get('capability_type')}")

        interfaces = card.get("interfaces", [])
        interface_ids = [item.get("id") for item in interfaces]
        if None in interface_ids or len(interface_ids) != len(set(interface_ids)):
            errors.append(f"{label}: missing or duplicate interface IDs")
        for index, interface in enumerate(interfaces):
            exact_keys(
                errors, f"{label}.interfaces[{index}]", interface,
                {"id", "role", "interaction", "protocol", "schema", "parameter_schema"},
            )
            if interface.get("role") not in {"functional_operation", "lifecycle_operation"}:
                errors.append(f"{label}: invalid interface role {interface.get('role')}")
            if interface.get("interaction") not in {"request_response", "managed_task"}:
                errors.append(f"{label}: invalid interaction {interface.get('interaction')}")
            if interface.get("protocol") not in {"ros2_topic", "ros2_service", "ros2_action"}:
                errors.append(f"{label}: invalid protocol {interface.get('protocol')}")

        lifecycle = card.get("lifecycle", {})
        exact_keys(errors, f"{label}.lifecycle", lifecycle, {"management", "operations"})
        if lifecycle.get("management") not in {"system_managed", "agent_managed", "shared_managed"}:
            errors.append(f"{label}: invalid lifecycle management")
        operations = lifecycle.get("operations", {})
        exact_keys(errors, f"{label}.lifecycle.operations", operations, {"start", "status", "stop"})
        for interface_id in operations.values():
            if interface_id is not None and interface_id not in interface_ids:
                errors.append(f"{label}: unknown lifecycle interface {interface_id}")

        for index, requirement in enumerate(card.get("hardware_requirements", [])):
            exact_keys(errors, f"{label}.hardware_requirements[{index}]", requirement, {"hardware_type"})
            if requirement.get("hardware_type") not in hardware_ids:
                errors.append(f"{label}: unknown hardware type {requirement.get('hardware_type')}")

        exact_keys(
            errors, f"{label}.resource_access", card.get("resource_access", {}),
            {"reads", "shared_writes", "exclusive_writes"},
        )

        if "demo" in card:
            exact_keys(errors, f"{label}.demo", card["demo"], {"representations"})
            for index, representation in enumerate(card["demo"].get("representations", [])):
                rep_label = f"{label}.demo.representations[{index}]"
                exact_keys(
                    errors, rep_label, representation,
                    {"id", "stage", "description", "interface", "renderer"},
                )
                if representation.get("stage") not in {"intermediate", "final"}:
                    errors.append(f"{rep_label}: invalid stage {representation.get('stage')}")
                exact_keys(errors, f"{rep_label}.interface", representation.get("interface"), {"protocol", "schema"})

    return errors, len(cards), len(capability_ids), len(hardware_ids)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("repository", nargs="?", default=".", help="AeroHub repository root")
    args = parser.parse_args()
    errors, cards, capabilities, hardware = validate(Path(args.repository).resolve())
    if errors:
        raise SystemExit("\n".join(errors))
    print(f"{cards} Cards, {capabilities} capability types, and {hardware} hardware types: OK")


if __name__ == "__main__":
    main()
