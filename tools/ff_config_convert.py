#!/usr/bin/env python3
"""Convert FF configuration between canonical JSON and human-editable CSV files."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

CHANNELS = ("CH1", "CH2", "CH3")
MOTION_FIELDS = ("position_tolerance", "stable_time_ms", "move_timeout_ms")
MOTOR_FIELDS = ("speed_limit_raw", "acceleration_raw", "deceleration_raw")
COORDINATE_UNITS = {"encoder_position": "count", "optical_position": "mm"}
SYSTEM_FIELDS = ("machine_id", "config_version", "max_active_rod_count", "status_expiration_ms")
CHANNEL_FIELDS = (
    "channel_id", "controller_name", "port", "baud_rate", "timeout_ms",
    "poll_period_ms", "retry_count", "training_rod_id",
)
ROD_FIELDS = (
    "channel_id", "rod_id", "name", "hard_min", "hard_max", "soft_min", "soft_max",
    "zero_position", "safe_position", "safe_position_valid", "P0", "P1",
    "inward_direction", "max_velocity", "max_acceleration", "max_deceleration",
    "position_tolerance", "stable_time_ms", "move_timeout_ms",
)
ROD_LAYER_FIELDS = ("channel_id", "rod_id", "layer", "role", "left_neighbor", "right_neighbor")
RANGE_FIELDS = ("channel_id", "rod_id", "range_kind", "min_position", "max_position")
PAIR_FIELDS = ("channel_id", "layer", "rod_a", "rod_b", "minimum_clearance_mm")
INT_FIELDS = {"max_active_rod_count", "status_expiration_ms", "baud_rate", "timeout_ms", "poll_period_ms", "retry_count", "training_rod_id", "rod_id", "rod_a", "rod_b", "layer", "role", "left_neighbor", "right_neighbor", "hard_min", "hard_max", "soft_min", "soft_max", "zero_position", "safe_position", "inward_direction", "position_tolerance", "stable_time_ms", "move_timeout_ms", "min_position", "max_position", *MOTOR_FIELDS}
FLOAT_FIELDS = {"P0", "P1", "max_velocity", "max_acceleration", "max_deceleration", "minimum_clearance_mm"}
BOOL_FIELDS = {"safe_position_valid"}


def fail(message: str) -> None:
    raise ValueError(message)


def parse_value(field: str, value: str) -> Any:
    value = value.strip()
    if value == "":
        return None
    try:
        if field in INT_FIELDS:
            return int(value)
        if field in FLOAT_FIELDS:
            return float(value)
        if field in BOOL_FIELDS:
            low = value.lower()
            if low in ("true", "1", "yes", "是"):
                return True
            if low in ("false", "0", "no", "否"):
                return False
            fail(f"字段 {field} 不是合法布尔值: {value}")
    except ValueError:
        fail(f"字段 {field} 不是合法数值: {value}")
    return value


def read_csv(path: Path, fields: tuple[str, ...]) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            fail(f"CSV 没有表头: {path}")
        missing = [f for f in fields if f not in reader.fieldnames]
        if missing:
            fail(f"{path.name} 缺少列: {', '.join(missing)}")
        return [{f: parse_value(f, row.get(f, "")) for f in fields} for row in reader]


def required(row: dict[str, Any], field: str, source: str) -> Any:
    value = row.get(field)
    if value is None or value == "":
        fail(f"{source}: 必填字段 {field} 为空")
    return value


def validate_config(config: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    motion = config.get("rod_motion")
    if motion is not None:
        if motion.get("coordinate_system") != "encoder_counts":
            errors.append("rod_motion.coordinate_system 必须为 encoder_counts")
        for field in MOTION_FIELDS:
            value = motion.get(field)
            if not isinstance(value, (int, float)) or value < 0:
                errors.append(f"rod_motion.{field} 必须为非负数")
    motor = config.get("motor_parameters")
    if motor is not None:
        for field, minimum, maximum in (("speed_limit_raw", 1, 1023), ("acceleration_raw", 0, 255), ("deceleration_raw", 0, 255)):
            value = motor.get(field)
            if not isinstance(value, int) or not minimum <= value <= maximum:
                errors.append(f"motor_parameters.{field} 必须在 {minimum}..{maximum} 内")
    if config.get("coordinate_units", COORDINATE_UNITS) != COORDINATE_UNITS:
        errors.append("coordinate_units 必须使用 count、mm")
    channels = config.get("channels")
    if not isinstance(channels, list) or not 1 <= len(channels) <= 3:
        errors.append("channels 必须包含 1..3 个不同的 CH")
        return errors
    seen_channels: set[str] = set()
    ports: set[str] = set()
    for ch in channels:
        cid = ch.get("id")
        if cid not in CHANNELS or cid in seen_channels:
            errors.append(f"Channel ID 非法或重复: {cid}")
        seen_channels.add(cid)
        serial = ch.get("controller", {}).get("serial", {})
        port = serial.get("port")
        if not port or port in ports:
            errors.append(f"串口为空或重复: {port}")
        ports.add(port)
        rods = ch.get("rods", [])
        seen_rods: set[int] = set()
        for rod in rods:
            rid = rod.get("id")
            if not isinstance(rid, int) or not 0 <= rid <= 255 or rid in seen_rods:
                errors.append(f"{cid} Rod ID 非法或重复: {rid}")
            seen_rods.add(rid)
            if rod.get("hard_min", 0) > rod.get("hard_max", 0):
                errors.append(f"{cid}/Rod{rid}: hard_min > hard_max")
            if not 0 <= rod.get("hard_min", 0) <= 4095 or not 0 <= rod.get("hard_max", 0) <= 4095:
                errors.append(f"{cid}/Rod{rid}: 设备行程限位必须在 0..4095 内")
            if rod.get("soft_min", 0) < rod.get("hard_min", 0) or rod.get("soft_max", 0) > rod.get("hard_max", 0) or rod.get("soft_min", 0) > rod.get("soft_max", 0):
                errors.append(f"{cid}/Rod{rid}: 软件限位不在硬限位内")
            if rod.get("safe_position_valid") and not (rod.get("soft_min", 0) <= rod.get("safe_position", 0) <= rod.get("soft_max", 0)):
                errors.append(f"{cid}/Rod{rid}: safe_position 超出软件限位")
        memberships = ch.get("layer_members", [])
        seen_memberships: set[tuple[int, int]] = set()
        by_member = {(item.get("layer"), item.get("rod_id")): item for item in memberships}
        if rods:
            assigned = {item.get("rod_id") for item in memberships}
            for rid in seen_rods - assigned:
                errors.append(f"{cid}/Rod{rid}: 缺少 layer_members")
        rules = ch.get("clearance_rules", [])
        rule_keys = {(rule.get("layer"), frozenset((rule.get("rod_a"), rule.get("rod_b"))))
                     for rule in rules}
        for item in memberships:
            layer, rid = item.get("layer"), item.get("rod_id")
            if not isinstance(layer, int) or not isinstance(rid, int) or rid not in seen_rods:
                errors.append(f"{cid}: layer member 非法: layer={layer}, rod={rid}")
                continue
            if (layer, rid) in seen_memberships:
                errors.append(f"{cid}: 同一 Rod 在同一 layer 重复: layer={layer}, rod={rid}")
            seen_memberships.add((layer, rid))
            role, left, right = item.get("role"), item.get("left_neighbor"), item.get("right_neighbor")
            has_left, has_right = left is not None and left >= 0, right is not None and right >= 0
            if role not in (0, 1, 2, 3) or (role == 0 and not (has_left and has_right)) or (role == 1 and (has_left or not has_right)) or (role == 2 and (not has_left or has_right)) or (role == 3 and (has_left or has_right)):
                errors.append(f"{cid}/layer {layer}/Rod{rid}: role 与 neighbor 不一致")
            if has_left and by_member.get((layer, left), {}).get("right_neighbor") != rid:
                errors.append(f"{cid}/layer {layer}/Rod{rid}: left_neighbor 不互反")
            if has_right and by_member.get((layer, right), {}).get("left_neighbor") != rid:
                errors.append(f"{cid}/layer {layer}/Rod{rid}: right_neighbor 不互反")
            if has_right and (layer, frozenset((rid, right))) not in rule_keys:
                errors.append(f"{cid}/layer {layer}/Rod{rid}: 缺少相邻杆间隙规则")
        training_rod = ch.get("training_rod_id")
        if training_rod is not None:
            if not isinstance(training_rod, int) or not 0 <= training_rod <= 255:
                errors.append(f"{cid}: training_rod_id 非法: {training_rod}")
            elif training_rod not in seen_rods:
                errors.append(f"{cid}: training_rod_id 引用了不存在的 Rod: {training_rod}")
            elif (2, training_rod) not in seen_memberships:
                errors.append(f"{cid}: training_rod_id 必须属于 Layer 2: {training_rod}")
    return errors


def csv_to_json(directory: Path) -> dict[str, Any]:
    system_rows = read_csv(directory / "system.csv", SYSTEM_FIELDS)
    channel_rows = read_csv(directory / "channels.csv", CHANNEL_FIELDS)
    rod_rows = read_csv(directory / "rods.csv", ROD_FIELDS)
    layer_rows = read_csv(directory / "rod_layers.csv", ROD_LAYER_FIELDS)
    range_rows = read_csv(directory / "safety_ranges.csv", RANGE_FIELDS)
    pair_rows = read_csv(directory / "pair_clearance_rules.csv", PAIR_FIELDS)
    motion_rows = read_csv(directory / "motion.csv", MOTION_FIELDS)
    motor_rows = read_csv(directory / "motor.csv", MOTOR_FIELDS)
    if motion_rows and len(motion_rows) != 1:
        fail("motion.csv 必须恰好有一行")
    if motor_rows and len(motor_rows) != 1:
        fail("motor.csv 必须恰好有一行")
    if len(system_rows) != 1:
        fail("system.csv 必须恰好有一行")
    if not 1 <= len(channel_rows) <= 3:
        fail("channels.csv 必须有 1..3 行")
    system = system_rows[0]
    channels: list[dict[str, Any]] = []
    for row in channel_rows:
        cid = required(row, "channel_id", "channels.csv")
        controller = {
            "name": required(row, "controller_name", "channels.csv"),
            "serial": {"port": required(row, "port", "channels.csv"), "baud_rate": row["baud_rate"], "timeout_ms": row["timeout_ms"]},
            "poll_period_ms": row["poll_period_ms"], "retry_count": row["retry_count"],
        }
        channels.append({"id": cid, "controller": controller,
                         "training_rod_id": row["training_rod_id"],
                         "layer_members": [], "clearance_rules": [], "rods": []})
    by_channel = {ch["id"]: ch for ch in channels}
    for row in rod_rows:
        cid = required(row, "channel_id", "rods.csv")
        if cid not in by_channel:
            fail(f"rods.csv 引用了不存在的 Channel: {cid}")
        rod = {"id": required(row, "rod_id", "rods.csv"), "name": row["name"]}
        for field in ROD_FIELDS[3:]:
            key = field
            if field.endswith("_ms"):
                key = field
            rod[key] = row[field]
        if motion_rows:
            for field in MOTION_FIELDS:
                if rod[field] != motion_rows[0][field]:
                    fail(f"{cid}/Rod{rod['id']}: {field} 与 motion.csv 不一致")
                del rod[field]
        if motor_rows:
            for field, old_field in (("speed_limit_raw", "max_velocity"), ("acceleration_raw", "max_acceleration"), ("deceleration_raw", "max_deceleration")):
                if rod[old_field] != motor_rows[0][field]:
                    fail(f"{cid}/Rod{rod['id']}: {old_field} 与 motor.csv 不一致")
                del rod[old_field]
        by_channel[cid]["rods"].append(rod)
    for row in layer_rows:
        ch = by_channel.get(row.get("channel_id"));
        if ch is None: fail(f"rod_layers.csv 引用了不存在的 Channel: {row.get('channel_id')}")
        ch["layer_members"].append({"layer": row["layer"], "rod_id": row["rod_id"], "role": row["role"], "left_neighbor": row["left_neighbor"], "right_neighbor": row["right_neighbor"]})
    for row in range_rows:
        ch = by_channel.get(row.get("channel_id"));
        if ch is None: fail(f"safety_ranges.csv 引用了不存在的 Channel: {row.get('channel_id')}")
        rod = next((r for r in ch["rods"] if r["id"] == row["rod_id"]), None)
        if rod is None: fail(f"safety_ranges.csv 引用了不存在的 Rod: {row.get('rod_id')}")
        rod.setdefault("safety_ranges", []).append({"kind": row["range_kind"], "min": row["min_position"], "max": row["max_position"]})
    for row in pair_rows:
        ch = by_channel.get(row.get("channel_id"));
        if ch is None: fail(f"pair_clearance_rules.csv 引用了不存在的 Channel: {row.get('channel_id')}")
        ch["clearance_rules"].append({"layer": row["layer"], "rod_a": row["rod_a"], "rod_b": row["rod_b"], "minimum_clearance_mm": row["minimum_clearance_mm"]})
    config = {"schema": "ff-config", "schema_version": 2, "machine_id": required(system, "machine_id", "system.csv"), "config_version": required(system, "config_version", "system.csv"), "max_active_rod_count": system["max_active_rod_count"], "status_expiration_ms": system["status_expiration_ms"], "channels": channels}
    if motion_rows:
        config["coordinate_units"] = COORDINATE_UNITS.copy()
        config["rod_motion"] = {"coordinate_system": "encoder_counts", **motion_rows[0]}
    if motor_rows:
        config["coordinate_units"] = COORDINATE_UNITS.copy()
        config["motor_parameters"] = motor_rows[0]
    errors = validate_config(config)
    if errors:
        fail("配置校验失败:\n- " + "\n- ".join(errors))
    return config


def write_csv(path: Path, fields: tuple[str, ...], rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows({field: row.get(field, "") for field in fields} for row in rows)


def json_to_csv(config: dict[str, Any], directory: Path) -> None:
    errors = validate_config(config)
    if errors:
        fail("JSON 配置校验失败:\n- " + "\n- ".join(errors))
    directory.mkdir(parents=True, exist_ok=True)
    write_csv(directory / "system.csv", SYSTEM_FIELDS, [{"machine_id": config.get("machine_id", ""), "config_version": config.get("config_version", ""), "max_active_rod_count": config.get("max_active_rod_count", 0), "status_expiration_ms": config.get("status_expiration_ms", 200)}])
    write_csv(directory / "motion.csv", MOTION_FIELDS,
              [config["rod_motion"]] if "rod_motion" in config else [])
    write_csv(directory / "motor.csv", MOTOR_FIELDS,
              [config["motor_parameters"]] if "motor_parameters" in config else [])
    channels = []
    rods = []
    layers = []
    ranges = []
    pairs = []
    for ch in config["channels"]:
        serial = ch.get("controller", {}).get("serial", {})
        channels.append({"channel_id": ch["id"], "controller_name": ch.get("controller", {}).get("name", ""), "port": serial.get("port", ""), "baud_rate": serial.get("baud_rate", 115200), "timeout_ms": serial.get("timeout_ms", 500), "poll_period_ms": ch.get("controller", {}).get("poll_period_ms", 20), "retry_count": ch.get("controller", {}).get("retry_count", 3), "training_rod_id": ch.get("training_rod_id")})
        for rod in ch.get("rods", []):
            row = {"channel_id": ch["id"], "rod_id": rod.get("id", ""), **rod}
            if "rod_motion" in config:
                row.update({field: config["rod_motion"][field] for field in MOTION_FIELDS})
            if "motor_parameters" in config:
                row.update({old: config["motor_parameters"][new] for old, new in (("max_velocity", "speed_limit_raw"), ("max_acceleration", "acceleration_raw"), ("max_deceleration", "deceleration_raw"))})
            rods.append(row)
        for member in ch.get("layer_members", []): layers.append({"channel_id": ch["id"], **member})
        for rod in ch.get("rods", []):
            for item in rod.get("safety_ranges", []): ranges.append({"channel_id": ch["id"], "rod_id": rod.get("id", ""), "range_kind": item.get("kind", ""), "min_position": item.get("min", ""), "max_position": item.get("max", "")})
        for item in ch.get("clearance_rules", []): pairs.append({"channel_id": ch["id"], "layer": item.get("layer", ""), "rod_a": item.get("rod_a", ""), "rod_b": item.get("rod_b", ""), "minimum_clearance_mm": item.get("minimum_clearance_mm", "")})
    write_csv(directory / "channels.csv", CHANNEL_FIELDS, channels)
    write_csv(directory / "rods.csv", ROD_FIELDS, rods)
    write_csv(directory / "rod_layers.csv", ROD_LAYER_FIELDS, layers)
    write_csv(directory / "safety_ranges.csv", RANGE_FIELDS, ranges)
    write_csv(directory / "pair_clearance_rules.csv", PAIR_FIELDS, pairs)


def format_config(config: dict[str, Any]) -> str:
    lines = ["{"]
    items = list(config.items())
    for index, (key, value) in enumerate(items):
        suffix = "," if index + 1 < len(items) else ""
        if key != "channels":
            lines.append("  " + json.dumps(key, ensure_ascii=False) + ": " + json.dumps(value, ensure_ascii=False) + suffix)
            continue
        lines.append('  "channels": [')
        for channel_index, channel in enumerate(value):
            channel_suffix = "," if channel_index + 1 < len(value) else ""
            lines.append("    {")
            fields = list(channel.items())
            for field_index, (field, field_value) in enumerate(fields):
                field_suffix = "," if field_index + 1 < len(fields) else ""
                if field in ("layer_members", "clearance_rules", "rods"):
                    lines.append("      " + json.dumps(field) + ": [")
                    for item_index, item in enumerate(field_value):
                        item_suffix = "," if item_index + 1 < len(field_value) else ""
                        lines.append("        " + json.dumps(item, ensure_ascii=False, separators=(", ", ": ")) + item_suffix)
                    lines.append("      ]" + field_suffix)
                else:
                    lines.append("      " + json.dumps(field) + ": " + json.dumps(field_value, ensure_ascii=False, separators=(", ", ": ")) + field_suffix)
            lines.append("    }" + channel_suffix)
        lines.append("  ]" + suffix)
    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="FF 配置 JSON/CSV 转换与校验")
    sub = parser.add_subparsers(dest="command", required=True)
    p_import = sub.add_parser("csv-to-json", help="CSV 配置目录生成 JSON")
    p_import.add_argument("csv_dir", type=Path); p_import.add_argument("json_file", type=Path)
    p_export = sub.add_parser("json-to-csv", help="JSON 导出 CSV 配置目录")
    p_export.add_argument("json_file", type=Path); p_export.add_argument("csv_dir", type=Path)
    p_validate = sub.add_parser("validate", help="校验 CSV 目录或 JSON 文件")
    p_validate.add_argument("input", type=Path)
    args = parser.parse_args()
    try:
        if args.command == "csv-to-json":
            config = csv_to_json(args.csv_dir)
            args.json_file.write_text(format_config(config), encoding="utf-8")
            print(f"已生成 {args.json_file}")
        elif args.command == "json-to-csv":
            json_to_csv(json.loads(args.json_file.read_text(encoding="utf-8")), args.csv_dir); print(f"已生成 {args.csv_dir}")
        else:
            if args.input.is_dir(): csv_to_json(args.input)
            else: validate_config(json.loads(args.input.read_text(encoding="utf-8")))
            print("配置校验通过")
        return 0
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr); return 1


if __name__ == "__main__":
    raise SystemExit(main())
