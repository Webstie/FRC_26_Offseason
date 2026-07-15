#!/usr/bin/env python3
"""Plot the FEED shot distance->speed lookup table.

Renders flywheel speed vs. distance and writes a CSV of the finely-interpolated curve. The table
here MUST stay in sync with `src/main/java/frc/robot/subsystems/shooter/FeedProfile.java`: linear
interpolation within the table, and linear EXTRAPOLATION (not clamping) beyond the min/max distance,
along the slope of the two nearest points. Unlike the goal shot (see shooter_lookup_plot.py), there's
no PID/bang-bang split -- a lobbed feed is one table.

Hood angle is NOT part of this table: every feed shot uses one fixed angle
(`Constants.FeedConfig.FEED_HOOD_ROTATIONS`), only flywheel speed varies with distance -- same
lob-shot pattern FRC2910 and FRC6328 both use for their cross-field pass.

Usage:
    python tools/feed_lookup_plot.py            # writes PNG + CSV next to this script
    python tools/feed_lookup_plot.py --show     # also opens an interactive window
"""

import argparse
import csv
import os

import numpy as np

# {distance_m, speed_rps} -- keep in sync with FeedProfile.TABLE.
TABLE = [
    (3.0, 35.0),
    (5.0, 42.0),
    (7.0, 48.0),
    (9.0, 55.0),
    (12.0, 63.0),
    (16.0, 72.0),
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

    # Extend half a meter past each end so the extrapolated (dashed) region is visible on the chart.
    fine = np.linspace(dist.min() - 0.5, dist.max() + 0.5, 200)
    speed_fine = lookup_with_extrapolation(fine, TABLE, 1)
    in_range = (fine >= dist.min()) & (fine <= dist.max())

    fig, ax_speed = plt.subplots(1, 1, figsize=(7, 5))

    ax_speed.plot(fine[in_range], speed_fine[in_range], "-", color="tab:green", label="interpolated")
    ax_speed.plot(fine[~in_range], speed_fine[~in_range], "--", color="tab:green", label="extrapolated")
    ax_speed.plot(dist, speed, "o", color="tab:green", label="table points")
    ax_speed.set_title("Feed flywheel speed vs. distance (hood fixed)")
    ax_speed.set_xlabel("Distance to feed corner (m)")
    ax_speed.set_ylabel("Flywheel speed (rotor rps)")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()

    fig.suptitle("Feed lookup: speed vs. distance (hood locked at FeedConfig.FEED_HOOD_ROTATIONS)")
    fig.tight_layout()

    png_path = os.path.join(OUT_DIR, "feed_lookup.png")
    fig.savefig(png_path, dpi=150)
    print(f"wrote {png_path}")

    csv_path = os.path.join(OUT_DIR, "feed_lookup.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["distance_m", "speed_rps"])
        for d, s in zip(fine, speed_fine):
            writer.writerow([f"{d:.3f}", f"{s:.3f}"])
    print(f"wrote {csv_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
