from __future__ import annotations
import argparse
from pathlib import Path

from .scenarios import get_scenario, available_scenarios
from .simulation import run_simulation
from .visualization import (
    plot_trajectory_comparison, plot_safety_margin, plot_control_activity,
    plot_qp_snapshot, plot_barrier_geometry, plot_architecture, animate_comparison,
)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Demo a HOCBF-QP steering safety filter.")
    p.add_argument("--scenario", choices=available_scenarios(), default="straight")
    p.add_argument("--output", default="assets/generated", help="Directory for figures and GIFs")
    p.add_argument("--no-animation", action="store_true", help="Skip GIF generation")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    scenario = get_scenario(args.scenario)
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)

    nominal = run_simulation(scenario, use_cbf=False)
    safe = run_simulation(scenario, use_cbf=True)

    stem = scenario.name.replace("-", "_")
    plot_trajectory_comparison(nominal, safe, scenario, out / f"{stem}_trajectory.png")
    plot_safety_margin(nominal, safe, out / f"{stem}_safety_margin.png")
    plot_control_activity(safe, scenario, out / f"{stem}_control_activity.png")
    plot_qp_snapshot(safe, scenario, out / f"{stem}_qp_snapshot.png")
    plot_barrier_geometry(scenario, out / f"{stem}_barrier_geometry.png")
    plot_architecture(out / "architecture.png")
    if not args.no_animation:
        animate_comparison(nominal, safe, scenario, out / f"{stem}_demo.gif")

    print(f"Scenario: {scenario.name}")
    print(f"Nominal collision: {nominal.collision}; minimum design clearance: {nominal.minimum_clearance:+.3f} m")
    print(f"CBF collision:     {safe.collision}; minimum design clearance: {safe.minimum_clearance:+.3f} m")
    print(f"CBF QP feasible at all sampled steps: {bool(safe.qp_feasible.all())}")
    print(f"Filter intervention fraction: {100*safe.intervention_fraction:.1f}%")
    print(f"Generated assets in: {out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
