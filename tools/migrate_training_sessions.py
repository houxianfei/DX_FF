"""Convert legacy FF-TRAIN-1 files to three-channel FF-TRAIN-2 archives."""

import argparse
from decimal import Decimal, ROUND_HALF_UP
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


FLOAT_FIELDS = {
    "start_mm", "end_mm", "step_mm", "prominence", "distance_mm",
    "position_mm", "requested_mm", "quantized_mm",
}


def rounded(value):
    number = Decimal(str(value)).quantize(Decimal("0.001"), rounding=ROUND_HALF_UP)
    return float(number) if number else 0


def normalize(value, key=""):
    if isinstance(value, dict):
        return {name: normalize(item, name) for name, item in value.items()}
    if isinstance(value, list):
        return [normalize(item, key) for item in value]
    if key in FLOAT_FIELDS or key == "candidates_mm" and isinstance(value, (int, float)):
        return rounded(value)
    return value


def convert(legacy):
    if legacy.get("schema") != "FF-TRAIN-1":
        raise ValueError("not a legacy training session")
    channel = legacy.get("channel")
    if type(channel) is not int or channel not in range(3):
        raise ValueError("invalid legacy channel")
    training = {key: value for key, value in legacy.items() if key not in {
        "schema", "machine_id", "config_version", "created_unix_seconds", "channel"
    }}
    training = normalize(training)
    if training.get("step_mm", 0) <= 0:
        raise ValueError("step_mm rounds to zero at 0.001 precision")
    return {
        "schema": "FF-TRAIN-2",
        "machine_id": legacy.get("machine_id", ""),
        "config_version": legacy.get("config_version", ""),
        "created_unix_seconds": legacy.get("created_unix_seconds", 0),
        "selected_channel": channel,
        "channels": [
            {"channel": index, "training": training if index == channel else {}}
            for index in range(3)
        ],
    }


def migrate(path, validator, env):
    with path.open(encoding="utf-8") as source:
        original = json.load(source)
    if original.get("schema") == "FF-TRAIN-2":
        return "already converted"
    converted = convert(original)
    backup = path.with_name(path.name + ".ff-train-1.bak")
    temporary = path.with_name(path.name + ".ff-train-2.tmp")
    if backup.exists() or temporary.exists():
        raise FileExistsError("backup or staging file already exists")
    try:
        with temporary.open("w", encoding="utf-8", newline="\n") as target:
            json.dump(converted, target, ensure_ascii=False, indent=2)
            target.write("\n")
            target.flush()
            os.fsync(target.fileno())
        with temporary.open(encoding="utf-8") as check:
            parsed = json.load(check)
        if len(parsed["channels"]) != 3 or parsed["selected_channel"] != original["channel"]:
            raise ValueError("round-trip archive structure differs")
        subprocess.run([str(validator), str(temporary)], check=True, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        shutil.copy2(path, backup)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return f"converted; backup: {backup.name}"


def main():
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path,
                        default=Path(os.environ["PROGRAMDATA"]) / "RSIC" / "FF" / "training_sessions")
    parser.add_argument("--validator", type=Path,
                        default=root / "x64" / "Release" / "validate_training_archive.exe")
    args = parser.parse_args()
    directory = args.directory.resolve(strict=True)
    validator = args.validator.resolve(strict=True)
    env = os.environ.copy()
    env["PATH"] = str(root / "FF_SDK" / "bin" / "x64" / "Release") + os.pathsep + env.get("PATH", "")
    failed = 0
    for path in sorted(directory.glob("*.json")):
        if not path.is_file() or path.is_symlink():
            continue
        try:
            print(f"{path.name}: {migrate(path, validator, env)}")
        except Exception as error:
            failed += 1
            print(f"{path.name}: failed: {error}", file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
