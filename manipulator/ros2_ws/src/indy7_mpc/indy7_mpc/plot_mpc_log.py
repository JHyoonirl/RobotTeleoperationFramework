#!/usr/bin/env python3
"""Create human-readable plots and a compact summary from an MPC JSONL run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


COST_NAMES = ("se3_tracking", "posture", "velocity", "torque", "joint_limit")


def _number(value, default=np.nan):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if np.isfinite(number) else default


def _vector(value, size=None):
    array = np.asarray(value if value is not None else [], dtype=float)
    if size is not None and array.size != size:
        return np.full(size, np.nan)
    return array.reshape(-1)


def _norm(value):
    array = _vector(value)
    return float(np.linalg.norm(array)) if array.size and np.all(np.isfinite(array)) else np.nan


def load_log(path: Path) -> list[dict]:
    if path.is_dir():
        path = path / "trace.jsonl"
    if not path.is_file():
        raise FileNotFoundError(f"trace.jsonl을 찾을 수 없습니다: {path}")
    records = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"JSONL {line_number}번째 줄이 잘못되었습니다.") from exc
            if isinstance(record, dict):
                records.append(record)
    if not records:
        raise ValueError(f"로그가 비어 있습니다: {path}")
    return records


def _series(records: list[dict]):
    time = np.asarray([_number(item.get("elapsed_s", i), i) for i, item in enumerate(records)])
    total = np.asarray([_number(item.get("solver", {}).get("cost")) for item in records])
    position = np.asarray([_number(item.get("position_error_norm_m")) for item in records])
    orientation = np.asarray([_number(item.get("orientation_error_norm_rad")) for item in records])
    path = np.asarray([_number(item.get("cartesian_path", {}).get("deviation_m")) for item in records])
    solve_ms = np.asarray([_number(item.get("solver", {}).get("solve_time_ms")) for item in records])
    success = np.asarray([float(bool(item.get("solver", {}).get("success", False))) for item in records])
    iterations = np.asarray([_number(item.get("solver", {}).get("iterations")) for item in records])
    torque_norm = np.asarray([_norm(item.get("control", {}).get("raw_torque", item.get("control", {}).get("raw_qddot"))) for item in records])
    velocity_norm = np.asarray([_norm(item.get("state", {}).get("velocity")) for item in records])
    acceleration_norm = np.asarray([_norm(item.get("dynamics", {}).get("predicted_acceleration")) for item in records])
    kinetic = np.asarray([_number(item.get("dynamics", {}).get("kinetic_energy")) for item in records])
    potential = np.asarray([_number(item.get("dynamics", {}).get("potential_energy")) for item in records])
    costs = {
        name: np.asarray([_number(item.get("cost_terms", {}).get(name, item.get("cost_terms", {}).get("acceleration", np.nan) if name == "torque" else np.nan)) for item in records])
        for name in COST_NAMES
    }
    costs["terminal"] = np.asarray([sum(_number(v, 0.0) for v in item.get("terminal_cost_terms", {}).values()) for item in records])
    current = np.asarray([item.get("state", {}).get("tcp_pose", {}).get("position", [np.nan] * 3) for item in records], dtype=float)
    reference = np.asarray([item.get("cartesian_path", {}).get("reference_pose", {}).get("position", [np.nan] * 3) for item in records], dtype=float)
    goal = np.asarray([item.get("target_pose", {}).get("position", [np.nan] * 3) for item in records], dtype=float)
    return locals()


def _finite_plot(axis, x, y, **kwargs):
    mask = np.isfinite(x) & np.isfinite(y)
    if np.any(mask):
        axis.plot(x[mask], y[mask], **kwargs)


def make_plots(records: list[dict], output_dir: Path, show: bool = False) -> dict:
    if not show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = _series(records)
    t = data["time"]
    output_dir.mkdir(parents=True, exist_ok=True)

    figure, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=True)
    ax_cost, ax_error, ax_solver, ax_dyn = axes.flat
    for name in (*COST_NAMES, "terminal"):
        _finite_plot(ax_cost, t, data["costs"][name], label=name)
    _finite_plot(ax_cost, t, data["total"], label="solver total", color="black", linewidth=1.5)
    ax_cost.set_title("Cost terms")
    ax_cost.set_ylabel("cost")
    ax_cost.legend(fontsize="small", ncol=2)
    _finite_plot(ax_error, t, data["position"], label="position [m]")
    _finite_plot(ax_error, t, np.degrees(data["orientation"]), label="orientation [deg]")
    _finite_plot(ax_error, t, data["path"], label="line deviation [m]")
    ax_error.set_title("Tracking")
    ax_error.set_ylabel("error")
    ax_error.legend(fontsize="small")
    _finite_plot(ax_solver, t, data["solve_ms"], label="solve [ms]")
    _finite_plot(ax_solver, t, data["iterations"], label="iterations")
    _finite_plot(ax_solver, t, data["success"], label="success", drawstyle="steps-post")
    ax_solver.set_title("Solver health")
    ax_solver.set_ylabel("value")
    ax_solver.set_ylim(bottom=0)
    ax_solver.legend(fontsize="small")
    _finite_plot(ax_dyn, t, data["torque_norm"], label="|tau|")
    _finite_plot(ax_dyn, t, data["acceleration_norm"], label="|ABA qddot|")
    _finite_plot(ax_dyn, t, data["velocity_norm"], label="|qdot|")
    _finite_plot(ax_dyn, t, data["kinetic"], label="kinetic energy")
    _finite_plot(ax_dyn, t, data["potential"], label="potential energy")
    ax_dyn.set_title("Dynamics")
    ax_dyn.set_ylabel("norm / energy")
    ax_dyn.legend(fontsize="small")
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.set_xlabel("time [s]")
    figure.suptitle(f"Indy7 MPC run ({len(records)} records)")
    figure.tight_layout()
    overview = output_dir / "overview.png"
    figure.savefig(overview, dpi=140)
    if show:
        plt.show(block=False)
    plt.close(figure)

    path_figure = plt.figure(figsize=(8, 7))
    path_axis = path_figure.add_subplot(111, projection="3d")
    for values, label, style in ((data["current"], "current TCP", "-"), (data["reference"], "reference", "--"), (data["goal"], "goal", ":")):
        if values.ndim == 2 and values.shape[1] == 3:
            mask = np.all(np.isfinite(values), axis=1)
            if np.any(mask):
                path_axis.plot(values[mask, 0], values[mask, 1], values[mask, 2], style, label=label)
    path_axis.set_title("TCP Cartesian path")
    path_axis.set_xlabel("x [m]")
    path_axis.set_ylabel("y [m]")
    path_axis.set_zlabel("z [m]")
    path_axis.legend()
    path_figure.tight_layout()
    path_plot = output_dir / "cartesian_path.png"
    path_figure.savefig(path_plot, dpi=140)
    plt.close(path_figure)
    return {"overview": overview, "cartesian_path": path_plot}


def write_summary(records: list[dict], output_dir: Path) -> Path:
    data = _series(records)

    def stats(values):
        finite = np.asarray(values)[np.isfinite(values)]
        if not finite.size:
            return {"final": None, "mean": None, "max": None}
        return {"final": float(finite[-1]), "mean": float(np.mean(finite)), "max": float(np.max(finite))}

    summary = {
        "records": len(records),
        "duration_s": float(data["time"][-1] - data["time"][0]),
        "status_counts": {},
        "cost_terms": {name: stats(data["costs"][name]) for name in (*COST_NAMES, "terminal")},
        "tracking": {"position_m": stats(data["position"]), "orientation_rad": stats(data["orientation"]), "line_deviation_m": stats(data["path"])},
        "solver": {"cost": stats(data["total"]), "solve_time_ms": stats(data["solve_ms"]), "iterations": stats(data["iterations"]), "success_rate": float(np.mean(data["success"]))},
        "dynamics": {"torque_norm": stats(data["torque_norm"]), "predicted_acceleration_norm": stats(data["acceleration_norm"]), "velocity_norm": stats(data["velocity_norm"]), "kinetic_energy": stats(data["kinetic"]), "potential_energy": stats(data["potential"])},
    }
    for item in records:
        status = str(item.get("status", "unknown"))
        summary["status_counts"][status] = summary["status_counts"].get(status, 0) + 1
    path = output_dir / "summary.json"
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="trace.jsonl 또는 실행 디렉터리")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--show", action="store_true", help="생성 후 matplotlib 창을 표시")
    options = parser.parse_args(args)
    log_path = options.log / "trace.jsonl" if options.log.is_dir() else options.log
    records = load_log(log_path)
    output_dir = options.output_dir or log_path.parent / "plots"
    plots = make_plots(records, output_dir, options.show)
    summary = write_summary(records, output_dir)
    print(f"records={len(records)}")
    print(f"overview={plots['overview']}")
    print(f"cartesian_path={plots['cartesian_path']}")
    print(f"summary={summary}")


if __name__ == "__main__":
    main()
