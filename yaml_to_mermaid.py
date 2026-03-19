#!/usr/bin/env python3

import argparse
import re
import sys
from typing import Dict, Iterable, List, Tuple

try:
    import yaml
except ModuleNotFoundError:
    yaml = None


Endpoint = Tuple[str, str]
Edge = Tuple[Endpoint, Endpoint]


def _sanitize_id(name: str) -> str:
    return "N_" + re.sub(r"[^A-Za-z0-9_]", "_", name)


def _escape_label(text: str) -> str:
    return text.replace('"', '\\"')


def _parse_endpoint_text(text: str) -> Endpoint:
    value = str(text).strip()
    if "." in value:
        comp, pin = value.split(".", 1)
        return comp.strip(), pin.strip()
    if ":" in value:
        comp, pin = value.split(":", 1)
        return comp.strip(), pin.strip()
    return value, ""


def _parse_endpoint_obj(item) -> Endpoint:
    if isinstance(item, dict) and len(item) == 1:
        component, pin = next(iter(item.items()))
        return str(component).strip(), str(pin).strip()
    if isinstance(item, str):
        return _parse_endpoint_text(item)
    raise ValueError(f"Unsupported endpoint format: {item!r}")


def _parse_connections(raw_connections) -> List[Edge]:
    edges: List[Edge] = []

    for item in raw_connections or []:
        if isinstance(item, dict):
            for left, right in item.items():
                edges.append((_parse_endpoint_text(left), _parse_endpoint_text(right)))
            continue

        if isinstance(item, list):
            endpoints = [_parse_endpoint_obj(part) for part in item]
            for index in range(len(endpoints) - 1):
                edges.append((endpoints[index], endpoints[index + 1]))
            continue

        raise ValueError(f"Unsupported connection entry: {item!r}")

    return edges


def _build_node_labels(
    connectors: Dict[str, dict], components: Iterable[str]
) -> Dict[str, str]:
    labels: Dict[str, str] = {}

    for component in components:
        info = connectors.get(component, {})
        if not isinstance(info, dict):
            info = {}

        details = []
        if info.get("type"):
            details.append(str(info["type"]))
        if info.get("subtype"):
            details.append(str(info["subtype"]))
        if info.get("value"):
            details.append(str(info["value"]))

        if details:
            labels[component] = f"{component}<br/>{' | '.join(details)}"
        else:
            labels[component] = component

    return labels


def to_mermaid(data: dict) -> str:
    connectors = data.get("connectors", {}) or {}
    if not isinstance(connectors, dict):
        raise ValueError("'connectors' must be a mapping")

    edges = _parse_connections(data.get("connections", []))

    components = set(connectors.keys())
    for (left_component, _), (right_component, _) in edges:
        components.add(left_component)
        components.add(right_component)

    labels = _build_node_labels(connectors, sorted(components))
    node_ids = {component: _sanitize_id(component) for component in sorted(components)}

    lines: List[str] = ["flowchart LR"]
    for component in sorted(components):
        lines.append(f'    {node_ids[component]}["{_escape_label(labels[component])}"]')

    if edges:
        lines.append("")

    for (left_component, left_pin), (right_component, right_pin) in edges:
        label = f"{left_pin} ↔ {right_pin}" if left_pin or right_pin else ""
        if label:
            lines.append(
                f'    {node_ids[left_component]} -- "{_escape_label(label)}" --> {node_ids[right_component]}'
            )
        else:
            lines.append(
                f"    {node_ids[left_component]} --> {node_ids[right_component]}"
            )

    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert wiring YAML to Mermaid flowchart"
    )
    parser.add_argument("input", help="Path to YAML input file")
    parser.add_argument(
        "-o", "--output", help="Path to output .mmd file (defaults to stdout)"
    )
    args = parser.parse_args()

    if yaml is None:
        sys.stderr.write(
            "Missing dependency: PyYAML. Install it with 'pip install pyyaml' and retry.\n"
        )
        return 2

    with open(args.input, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    mermaid = to_mermaid(data)

    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(mermaid)
    else:
        sys.stdout.write(mermaid)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
