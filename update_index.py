#!/usr/bin/env python3
"""
Update Index
------------
Rebuilds the top-level README.md's drives table by injecting it into
a template file (README_template.md) at a marked placeholder, so the
rest of the README (project overview, architecture, engineering
writeup) survives being regenerated every time a new drive is added --
only the table itself gets replaced.

If README_template.md doesn't exist, falls back to writing a minimal
README with just the table (the original behavior), so this still
works standalone.

Usage:
    python3 update_index.py [--drives-dir drives] [--template README_template.md]
"""

import argparse
import json
from pathlib import Path

TABLE_PLACEHOLDER = "{{DRIVES_TABLE}}"


def build_table(drives_dir: Path) -> str:
    entries = []
    if drives_dir.exists():
        for session_dir in sorted(drives_dir.iterdir()):
            stats_path = session_dir / "stats.json"
            if not stats_path.exists():
                continue
            with open(stats_path) as f:
                stats = json.load(f)
            stats["_dir"] = session_dir.name
            entries.append(stats)

    entries.sort(key=lambda e: e["start_time"], reverse=True)  # newest first

    lines = [
        f"**{len(entries)} drive(s) recorded.**",
        "",
        "| Drive | Date | Duration | Distance | Max Speed | Hard Brakes |",
        "|---|---|---|---|---|---|",
    ]
    for e in entries:
        drive_dir = e["_dir"]
        date_str = e["start_time"].split("T")[0]
        duration_min = e["duration_s"] / 60
        distance = e.get("distance_miles", 0)
        max_speed = e.get("max_speed_mph", "n/a")
        brakes = e.get("num_hard_brake_events", 0)
        lines.append(
            f"| [{drive_dir}](drives/{drive_dir}/README.md) | {date_str} | "
            f"{duration_min:.1f} min | {distance:.1f} mi | {max_speed} mph | {brakes} |"
        )
    return "\n".join(lines), len(entries)


def main():
    parser = argparse.ArgumentParser(description="Rebuild the top-level README's drives table.")
    parser.add_argument("--drives-dir", default="drives", help="Directory containing per-drive folders (default: drives)")
    parser.add_argument("--template", default="README_template.md", help="Template file with a {{DRIVES_TABLE}} placeholder")
    parser.add_argument("--out", default="README.md", help="Output path for the top-level README (default: README.md)")
    args = parser.parse_args()

    table, count = build_table(Path(args.drives_dir))

    template_path = Path(args.template)
    if template_path.exists():
        content = template_path.read_text()
        if TABLE_PLACEHOLDER not in content:
            print(f"Warning: {args.template} has no {TABLE_PLACEHOLDER} placeholder -- appending table at the end.")
            content = content.rstrip() + "\n\n" + table + "\n"
        else:
            content = content.replace(TABLE_PLACEHOLDER, table)
    else:
        # Fallback: no template found, just write a minimal README.
        content = (
            "# Tacoma Telemetry & Sensor-Fusion Highway 17 Logger\n\n"
            + table + "\n"
        )

    Path(args.out).write_text(content)
    print(f"Wrote {args.out} with {count} drive(s)")


if __name__ == "__main__":
    main()

