#!/usr/bin/env python3
"""Plot the shooter distance->setpoint lookup table.

Renders flywheel speed vs. distance and hood position vs. distance, and writes a CSV of the
finely-interpolated curve. The table here MUST stay in sync with
`src/main/java/frc/robot/subsystems/shooter/ShooterProfile.java`: linear interpolation within the
table, and linear EXTRAPOLATION (not clamping) beyond the overall min/max distance, along the slope
of the two nearest points.

Usage:
    python tools/shooter_lookup_plot.py            # writes PNG + CSV next to this script
    python tools/shooter_lookup_plot.py --show     # also opens an interactive window
"""

import argparse
import csv
import os

import numpy as np

# {distance_m, speed_rps, hood_rotations} -- keep in sync with ShooterProfile.TABLE.
TABLE = [
    (1.25, 47.5, 0.4),
    (1.75, 50.0, 0.5),
    (2.25, 51.0, 0.6),
    (3.0, 52.0, 0.7),
    (3.75, 52.0, 0.7),
]

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def extrapolate(a, b, x, column):
    """Linearly extrapolate along the line through a=(dist,...) and b=(dist,...) for `column`."""
    slope = (b[column] - a[column]) / (b[0] - a[0])
    return a[column] + slope * (x - a[0])


def lookup_with_extrapolation(x, table, column):
    """np.interp, but linearly extrapolating past the table's ends instead of clamping."""
    dist = np.array([row[0] for row in table])
    values = np.array([row[column] for row in table])
    y = np.interp(x, dist, values)
    below = x < dist.min()
    above = x > dist.max()
    y[below] = extrapolate(table[0], table[1], x[below], column)
    y[above] = extrapolate(table[-2], table[-1], x[above], column)
    return y


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action="store_true", help="open an interactive window")
    args = parser.parse_args()

    import matplotlib

    if not args.show:
        matplotlib.use("Agg")  # headless: render straight to file
    import matplotlib.pyplot as plt

    dist = np.array([row[0] for row in TABLE])
    speed = np.array([row[1] for row in TABLE])
    hood = np.array([row[2] for row in TABLE])

    # Extend half a meter past each end so the extrapolated (dashed) region is visible on the chart.
    fine = np.linspace(dist.min() - 0.5, dist.max() + 0.5, 200)
    speed_fine = lookup_with_extrapolation(fine, TABLE, 1)
    hood_fine = lookup_with_extrapolation(fine, TABLE, 2)
    in_range = (fine >= dist.min()) & (fine <= dist.max())

    fig, (ax_speed, ax_hood) = plt.subplots(1, 2, figsize=(12, 5))

    ax_speed.plot(fine[in_range], speed_fine[in_range], "-", color="tab:blue", label="interpolated")
    ax_speed.plot(fine[~in_range], speed_fine[~in_range], "--", color="tab:blue", label="extrapolated")
    ax_speed.plot(dist, speed, "o", color="tab:blue", label="table points")
    ax_speed.set_title("Flywheel speed vs. distance")
    ax_speed.set_xlabel("Distance to goal (m)")
    ax_speed.set_ylabel("Flywheel speed (rotor rps)")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()

    ax_hood.plot(fine[in_range], hood_fine[in_range], "-", color="tab:orange", label="interpolated")
    ax_hood.plot(fine[~in_range], hood_fine[~in_range], "--", color="tab:orange", label="extrapolated")
    ax_hood.plot(dist, hood, "o", color="tab:orange", label="table points")
    ax_hood.set_title("Hood position vs. distance")
    ax_hood.set_xlabel("Distance to goal (m)")
    ax_hood.set_ylabel("Hood position (rotations)")
    ax_hood.grid(True, alpha=0.3)
    ax_hood.legend()

    fig.suptitle("Shooter lookup: speed & hood position vs. distance")
    fig.tight_layout()

    png_path = os.path.join(OUT_DIR, "shooter_lookup.png")
    fig.savefig(png_path, dpi=150)
    print(f"wrote {png_path}")

    csv_path = os.path.join(OUT_DIR, "shooter_lookup.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["distance_m", "speed_rps", "hood_rotations"])
        for d, s, h in zip(fine, speed_fine, hood_fine):
            writer.writerow([f"{d:.3f}", f"{s:.3f}", f"{h:.3f}"])
    print(f"wrote {csv_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
