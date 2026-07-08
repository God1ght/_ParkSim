import argparse
import bisect
import json
from pathlib import Path
from typing import Dict, List


def load_records(run_dir: Path) -> List[Dict[str, object]]:
    frames_dir = run_dir / "frames"
    meta_path = frames_dir / "frame_times.jsonl"
    records: List[Dict[str, object]] = []
    if meta_path.exists():
        for line in meta_path.read_text().splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            frame_path = frames_dir / str(row.get("frame", ""))
            if frame_path.exists():
                records.append({"sim_time": float(row.get("sim_time", 0.0)), "path": frame_path})
    else:
        for idx, frame_path in enumerate(sorted(frames_dir.glob("frame_*.png"))):
            records.append({"sim_time": float(idx), "path": frame_path})
    records.sort(key=lambda row: float(row["sim_time"]))
    return records


def nearest_record(records: List[Dict[str, object]], time_value: float) -> Dict[str, object]:
    times = [float(row["sim_time"]) for row in records]
    pos = bisect.bisect_left(times, time_value)
    candidates = []
    if pos < len(records):
        candidates.append(records[pos])
    if pos > 0:
        candidates.append(records[pos - 1])
    if not candidates:
        raise ValueError("no frame records available")
    return min(candidates, key=lambda row: abs(float(row["sim_time"]) - time_value))


def resize_to_width(image, width: int):
    from PIL import Image

    if image.mode != "RGB":
        image = image.convert("RGB")
    if image.width == width:
        return image
    height = max(1, int(round(image.height * width / image.width)))
    resample = getattr(getattr(Image, "Resampling", Image), "LANCZOS", Image.BICUBIC)
    return image.resize((width, height), resample)


def render_pair(left_path: Path, right_path: Path, out_path: Path, panel_width: int, label: str) -> None:
    from PIL import Image, ImageDraw

    left = resize_to_width(Image.open(left_path), panel_width)
    right = resize_to_width(Image.open(right_path), panel_width)
    height = max(left.height, right.height)
    label_h = 28
    canvas_height = height + label_h
    if canvas_height % 2:
        canvas_height += 1
    canvas = Image.new("RGB", (panel_width * 2, canvas_height), (0, 0, 0))
    canvas.paste(left, (0, label_h + (height - left.height) // 2))
    canvas.paste(right, (panel_width, label_h + (height - right.height) // 2))
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 7), "rule_based | %s" % label, fill=(255, 255, 255))
    draw.text((panel_width + 8, 7), "qwen_vla | %s" % label, fill=(255, 255, 255))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)


def align_frames(left_dir: Path, right_dir: Path, out_dir: Path, dt: float, panel_width: int) -> int:
    left_records = load_records(left_dir)
    right_records = load_records(right_dir)
    if not left_records or not right_records:
        raise ValueError("both runs must contain visualizer frames")
    start = max(float(left_records[0]["sim_time"]), float(right_records[0]["sim_time"]))
    end = min(float(left_records[-1]["sim_time"]), float(right_records[-1]["sim_time"]))
    if end < start:
        raise ValueError("frame time ranges do not overlap")
    idx = 0
    time_value = start
    while time_value <= end + 1e-9:
        left = nearest_record(left_records, time_value)
        right = nearest_record(right_records, time_value)
        render_pair(
            Path(left["path"]),
            Path(right["path"]),
            out_dir / ("frame_%06d.png" % idx),
            panel_width,
            "sim_time=%.2fs" % time_value,
        )
        idx += 1
        time_value = start + idx * float(dt)
    (out_dir / "alignment.json").write_text(json.dumps({
        "start_sim_time": start,
        "end_sim_time": end,
        "dt": float(dt),
        "frames": idx,
        "left_dir": str(left_dir),
        "right_dir": str(right_dir),
    }, indent=2))
    return idx


def main() -> None:
    parser = argparse.ArgumentParser(description="Align ParkSim visualizer frames by simulation time.")
    parser.add_argument("--left", required=True, type=Path)
    parser.add_argument("--right", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--panel-width", type=int, default=960)
    args = parser.parse_args()
    count = align_frames(args.left, args.right, args.out_dir, args.dt, args.panel_width)
    print("aligned_frames=%d out_dir=%s" % (count, args.out_dir))


if __name__ == "__main__":
    main()
