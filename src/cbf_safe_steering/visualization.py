from __future__ import annotations
from pathlib import Path
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch
from matplotlib.animation import FuncAnimation, PillowWriter

from .scenarios import Scenario
from .simulation import SimulationTrace


def _decorate_world(ax, scenario: Scenario, show_margin: bool = True):
    for obs in scenario.obstacles:
        physical_r = obs.radius + scenario.vehicle.vehicle_radius
        safe_r = physical_r + scenario.cbf.safety_margin
        ax.add_patch(Circle((obs.x, obs.y), obs.radius, alpha=0.35, label="Obstacle" if obs is scenario.obstacles[0] else None))
        ax.add_patch(Circle((obs.x, obs.y), physical_r, fill=False, linewidth=1.2, linestyle="-", alpha=0.7))
        if show_margin:
            ax.add_patch(Circle((obs.x, obs.y), safe_r, fill=False, linewidth=1.5, linestyle="--", alpha=0.8,
                                label="CBF safety boundary" if obs is scenario.obstacles[0] else None))
        ax.text(obs.x, obs.y, obs.label, ha="center", va="center", fontsize=8)
    ax.scatter([scenario.start.x], [scenario.start.y], marker="o", s=45, label="Start")
    ax.scatter([scenario.goal[0]], [scenario.goal[1]], marker="*", s=120, label="Goal")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def plot_trajectory_comparison(nominal: SimulationTrace, safe: SimulationTrace, scenario: Scenario, output: str | Path):
    fig, ax = plt.subplots(figsize=(10, 5.6))
    _decorate_world(ax, scenario)
    ax.plot(nominal.x, nominal.y, linewidth=2.0, label="Nominal controller")
    ax.plot(safe.x, safe.y, linewidth=2.4, label="Nominal + HOCBF-QP")

    skip = max(1, len(safe.x)//12)
    ax.quiver(safe.x[::skip], safe.y[::skip], np.cos(safe.psi[::skip]), np.sin(safe.psi[::skip]),
              angles='xy', scale_units='xy', scale=2.5, width=0.003, alpha=0.55)
    ax.set_title("Trajectory comparison: the safety filter minimally modifies steering")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_safety_margin(nominal: SimulationTrace, safe: SimulationTrace, output: str | Path):
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(nominal.t, nominal.min_clearance, linewidth=2, label="Nominal")
    ax.plot(safe.t, safe.min_clearance, linewidth=2.3, label="CBF filtered")
    ax.axhline(0.0, linestyle="--", linewidth=1.4, label="Design safety boundary")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Minimum clearance to CBF boundary [m]")
    ax.set_title("Safety margin over time")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_control_activity(safe: SimulationTrace, scenario: Scenario, output: str | Path):
    fig, axes = plt.subplots(3, 1, figsize=(9, 8.2), sharex=True)
    axes[0].plot(safe.t, safe.omega_nominal, label="Nominal yaw rate")
    axes[0].plot(safe.t, safe.omega_applied, linewidth=2, label="QP-safe yaw rate")
    axes[0].fill_between(safe.t, safe.qp_lower, safe.qp_upper, alpha=0.12, label="QP feasible interval")
    axes[0].set_ylabel(r"$\omega$ [rad/s]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(safe.t, np.degrees(safe.steer_applied), linewidth=2)
    axes[1].axhline(math.degrees(scenario.vehicle.max_steer), linestyle="--", alpha=0.7)
    axes[1].axhline(-math.degrees(scenario.vehicle.max_steer), linestyle="--", alpha=0.7)
    axes[1].set_ylabel(r"$\delta$ [deg]")
    axes[1].set_title("Applied steering angle and actuator bounds")
    axes[1].grid(True, alpha=0.25)

    axes[2].step(safe.t, safe.qp_active.astype(float), where="post", label="Safety filter active")
    axes[2].step(safe.t, (~safe.qp_feasible).astype(float), where="post", label="QP infeasible")
    axes[2].set_ylim(-0.08, 1.15)
    axes[2].set_yticks([0,1], ["No", "Yes"])
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Status")
    axes[2].grid(True, alpha=0.25)
    axes[2].legend(loc="best")
    fig.suptitle("What the real-time safety filter is doing")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_qp_snapshot(safe: SimulationTrace, scenario: Scenario, output: str | Path):
    indices = np.flatnonzero(safe.qp_active & safe.qp_feasible)
    idx = int(indices[len(indices)//2]) if len(indices) else len(safe.t)//2
    nominal = safe.omega_nominal[idx]
    chosen = safe.omega_applied[idx]
    lower, upper = safe.qp_lower[idx], safe.qp_upper[idx]
    wmax = scenario.vehicle.max_yaw_rate
    grid = np.linspace(-wmax, wmax, 500)
    objective = 0.5 * (grid - nominal)**2

    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    ax.plot(grid, objective, linewidth=2, label=r"$J(\omega)=\frac{1}{2}(\omega-\omega_{nom})^2$")
    if lower <= upper:
        ax.axvspan(max(lower, -wmax), min(upper, wmax), alpha=0.18, label="CBF-feasible yaw rates")
    ax.axvline(nominal, linestyle=":", linewidth=2, label="Nominal command")
    ax.axvline(chosen, linestyle="--", linewidth=2, label="QP solution")
    ax.set_xlabel(r"Yaw rate $\omega$ [rad/s]")
    ax.set_ylabel("QP objective")
    ax.set_title(f"One real-time QP snapshot at t = {safe.t[idx]:.2f} s")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_barrier_geometry(scenario: Scenario, output: str | Path):
    obs = scenario.obstacles[0]
    R = obs.radius + scenario.vehicle.vehicle_radius + scenario.cbf.safety_margin
    fig, ax = plt.subplots(figsize=(7.4, 6.2))
    ax.add_patch(Circle((obs.x, obs.y), obs.radius, alpha=0.35))
    ax.add_patch(Circle((obs.x, obs.y), obs.radius + scenario.vehicle.vehicle_radius, fill=False, linewidth=1.5))
    ax.add_patch(Circle((obs.x, obs.y), R, fill=False, linestyle="--", linewidth=2.0))

    theta = math.radians(205)
    px, py = obs.x + (R+0.65)*math.cos(theta), obs.y + (R+0.65)*math.sin(theta)
    ax.scatter([px], [py], s=70)
    ax.annotate("vehicle center", (px,py), xytext=(px-2.2, py-0.9), arrowprops=dict(arrowstyle="->"))
    ax.annotate(r"$h(x,y)=0$", (obs.x + R, obs.y), xytext=(obs.x + R + 0.6, obs.y + 0.7), arrowprops=dict(arrowstyle="->"))
    ax.text(obs.x, obs.y, "obstacle", ha="center", va="center")
    ax.text(obs.x, obs.y + R + 0.45, r"Safe set: $h(x,y)\geq 0$", ha="center", fontsize=12)
    ax.text(obs.x, obs.y - 0.15, r"Unsafe: $h<0$", ha="center", va="top", fontsize=11)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(obs.x-R-2.2, obs.x+R+2.2)
    ax.set_ylim(obs.y-R-1.5, obs.y+R+1.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Barrier-function geometry")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_architecture(output: str | Path):
    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.axis("off")
    boxes = [
        (0.03, 0.35, 0.18, 0.3, "Goal / reference"),
        (0.29, 0.35, 0.18, 0.3, "Nominal controller\n" + r"$\omega_{nom}$"),
        (0.55, 0.35, 0.18, 0.3, "HOCBF + QP\nsafety filter"),
        (0.81, 0.35, 0.16, 0.3, "Vehicle\n" + r"$\omega_{safe}$"),
    ]
    for x,y,w,h,text in boxes:
        patch = FancyBboxPatch((x,y), w,h, boxstyle="round,pad=0.02", linewidth=1.5, fill=False)
        ax.add_patch(patch)
        ax.text(x+w/2, y+h/2, text, ha="center", va="center", fontsize=11)
    for i in range(len(boxes)-1):
        x,y,w,h,_=boxes[i]; xn,yn,wn,hn,_=boxes[i+1]
        ax.annotate("", xy=(xn, yn+hn/2), xytext=(x+w, y+h/2), arrowprops=dict(arrowstyle="->", linewidth=1.8))
    ax.text(0.64, 0.12, "Obstacle states + safety constraints", ha="center", fontsize=10)
    ax.annotate("", xy=(0.64,0.34), xytext=(0.64,0.18), arrowprops=dict(arrowstyle="->", linewidth=1.5))
    ax.set_title("Safety filter architecture: performance first, intervention only when necessary", fontsize=13)
    fig.tight_layout()
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def animate_comparison(nominal: SimulationTrace, safe: SimulationTrace, scenario: Scenario, output: str | Path, fps: int = 24):
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.8), sharex=True, sharey=True)
    for ax, title in zip(axes, ["Nominal controller", "Nominal + HOCBF-QP"]):
        _decorate_world(ax, scenario)
        ax.set_title(title)
    allx = np.concatenate([nominal.x, safe.x, np.array([scenario.goal[0]])])
    ally = np.concatenate([nominal.y, safe.y, np.array([scenario.goal[1]])])
    xmin, xmax = allx.min()-1, allx.max()+1
    ymin, ymax = ally.min()-2, ally.max()+2
    for ax in axes:
        ax.set_xlim(xmin,xmax); ax.set_ylim(ymin,ymax)
        leg=ax.get_legend()
        if leg: leg.remove()

    line_n, = axes[0].plot([], [], linewidth=2)
    line_s, = axes[1].plot([], [], linewidth=2)
    dot_n, = axes[0].plot([], [], marker='o')
    dot_s, = axes[1].plot([], [], marker='o')
    text_n = axes[0].text(0.02,0.96,"", transform=axes[0].transAxes, va="top")
    text_s = axes[1].text(0.02,0.96,"", transform=axes[1].transAxes, va="top")

    nframes = max(len(nominal.t), len(safe.t))
    stride = max(1, int(round((1.0/max(scenario.dt,1e-6))/fps)))
    frames = list(range(0,nframes,stride))
    if frames[-1] != nframes-1: frames.append(nframes-1)

    def update(frame):
        i=min(frame,len(nominal.t)-1); j=min(frame,len(safe.t)-1)
        line_n.set_data(nominal.x[:i+1],nominal.y[:i+1]); dot_n.set_data([nominal.x[i]],[nominal.y[i]])
        line_s.set_data(safe.x[:j+1],safe.y[:j+1]); dot_s.set_data([safe.x[j]],[safe.y[j]])
        text_n.set_text(f"clearance = {nominal.min_clearance[i]:+.2f} m")
        text_s.set_text(f"clearance = {safe.min_clearance[j]:+.2f} m\nfilter active = {bool(safe.qp_active[j])}")
        return line_n,line_s,dot_n,dot_s,text_n,text_s

    ani=FuncAnimation(fig,update,frames=frames,interval=1000/fps,blit=False)
    ani.save(output,writer=PillowWriter(fps=fps),dpi=105)
    plt.close(fig)
