import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Dict


def export_profile(source: Path, destination: Path) -> Dict[str, Any]:
    source_bytes = source.read_bytes()
    result = json.loads(source_bytes)
    points = []
    for item in result["distance_profile"]:
        points.append({
            "x": item["x"],
            "y": item["y"],
            "s_m": item["s_m"],
            "ds_m": item["ds_m"],
            "target_speed_kmh": item["stability_capped_planned_speed_kmh"],
            "acceleration_mps2": item["observed_acceleration_envelope_mps2"],
            "deceleration_mps2": item["observed_deceleration_envelope_mps2"],
            "active_constraint": item["active_constraint"],
            "confidence": item["confidence"],
            "support": item["longitudinal_envelope_support"],
        })
    profile = {
        "schema_version": 1,
        "source_result": str(source),
        "source_result_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "point_count": len(points),
        "points": points,
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
    return profile


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a compact runtime shadow velocity profile")
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    profile = export_profile(args.source, args.destination)
    print(json.dumps({
        "destination": str(args.destination),
        "point_count": profile["point_count"],
        "source_result_sha256": profile["source_result_sha256"],
    }, indent=2))


if __name__ == "__main__":
    main()
