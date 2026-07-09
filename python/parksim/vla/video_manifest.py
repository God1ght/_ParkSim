import argparse
import json
from pathlib import Path
from typing import Any, Dict


def _line_count(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open() as f:
        return sum(1 for _ in f)


def _file_info(path: Path) -> Dict[str, Any]:
    return {
        "path": str(path),
        "exists": path.exists(),
        "bytes": path.stat().st_size if path.exists() else 0,
    }


def build_manifest(out_dir: Path) -> Dict[str, Any]:
    modes = [path.name for path in sorted(out_dir.iterdir()) if path.is_dir() and (path / "frames").exists()]
    mode_payload: Dict[str, Any] = {}
    for mode in modes:
        run_dir = out_dir / mode
        frames_dir = run_dir / "frames"
        mode_payload[mode] = {
            "frames_dir": str(frames_dir),
            "frame_png_count": len(list(frames_dir.glob("frame_*.png"))),
            "frame_times_count": _line_count(frames_dir / "frame_times.jsonl"),
            "mp4": _file_info(run_dir / (mode + ".mp4")),
            "gif": _file_info(run_dir / (mode + ".gif")),
            "logs_dir": str(run_dir / "logs"),
            "decision_log": _file_info(run_dir / "logs" / "qwen_vla_decisions.jsonl"),
        }
    payload = {
        "type": "parksim_vla_visualizer_videos",
        "out_dir": str(out_dir),
        "modes": mode_payload,
        "side_by_side": {
            "mp4": _file_info(out_dir / "rule_vs_qwen_vla.mp4"),
            "gif": _file_info(out_dir / "rule_vs_qwen_vla.gif"),
            "alignment": _file_info(out_dir / "aligned_rule_vs_qwen_vla" / "alignment.json"),
            "aligned_frame_count": len(list((out_dir / "aligned_rule_vs_qwen_vla").glob("frame_*.png"))),
        },
        "metrics": {
            "summary_md": _file_info(out_dir / "summary.md"),
            "metrics_csv": _file_info(out_dir / "metrics.csv"),
            "metrics_json": _file_info(out_dir / "metrics.json"),
            "trajectories_png": _file_info(out_dir / "trajectories.png"),
            "metrics_png": _file_info(out_dir / "metrics.png"),
        },
    }
    manifest_path = out_dir / "video_manifest.json"
    with manifest_path.open("w") as f:
        json.dump(payload, f, indent=2)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Write a manifest for ParkSim visualizer video artifacts.")
    parser.add_argument("out_dir", type=Path)
    args = parser.parse_args()
    payload = build_manifest(args.out_dir.resolve())
    print("video_manifest=%s modes=%d" % (args.out_dir.resolve() / "video_manifest.json", len(payload["modes"])))


if __name__ == "__main__":
    main()
