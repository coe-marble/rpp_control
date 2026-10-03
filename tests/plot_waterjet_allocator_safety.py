from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as pyplot
import numpy


def parse_arguments(arguments: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot the waterjet allocator safety-grid CSV.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("."))
    return parser.parse_args(arguments)


def read_rows(csv_path: Path) -> list[dict[str, float]]:
    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        return [
            {name: float(value) for name, value in row.items()}
            for row in csv.DictReader(csv_file)
        ]


def values_as_grid(
    rows: list[dict[str, float]], field: str
) -> tuple[numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    surge_values = numpy.array(sorted({row["surge_normalized"] for row in rows}))
    yaw_values = numpy.array(sorted({row["yaw_normalized"] for row in rows}))
    values = numpy.empty((yaw_values.size, surge_values.size))
    for row in rows:
        surge_index = int(numpy.where(surge_values == row["surge_normalized"])[0][0])
        yaw_index = int(numpy.where(yaw_values == row["yaw_normalized"])[0][0])
        values[yaw_index, surge_index] = row[field]
    return surge_values, yaw_values, values


def plot_maps(
    rows: list[dict[str, float]],
    fields: list[tuple[str, str]],
    output_path: Path,
) -> None:
    figure, axes = pyplot.subplots(1, len(fields), figsize=(6 * len(fields), 5))
    for axis, (field, title) in zip(numpy.atleast_1d(axes), fields):
        surge, yaw, values = values_as_grid(rows, field)
        image = axis.imshow(
            values,
            origin="lower",
            aspect="auto",
            extent=(surge.min(), surge.max(), yaw.min(), yaw.max()),
        )
        axis.set_title(title)
        axis.set_xlabel("normalized surge wrench")
        axis.set_ylabel("normalized yaw wrench")
        figure.colorbar(image, ax=axis)
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    pyplot.close(figure)


def main(arguments: Sequence[str] | None = None) -> None:
    parsed_arguments = parse_arguments(arguments)
    parsed_arguments.output_dir.mkdir(parents=True, exist_ok=True)
    rows = read_rows(parsed_arguments.csv_path)
    if not rows:
        raise ValueError("The safety-grid CSV is empty.")

    plot_maps(
        rows,
        [
            ("thrust_command_normalized", "normalized thrust command"),
            ("nozzle_command_normalized", "normalized nozzle command"),
            ("saturated", "saturation flag"),
        ],
        parsed_arguments.output_dir / "waterjet_allocator_commands.png",
    )
    plot_maps(
        rows,
        [
            ("realized_surge_n", "realized surge force [N]"),
            ("realized_yaw_nm", "realized yaw wrench [Nm]"),
        ],
        parsed_arguments.output_dir / "waterjet_allocator_realized_wrench.png",
    )
    plot_maps(
        rows,
        [
            ("surge_error_n", "surge residual [N]"),
            ("yaw_error_nm", "yaw residual [Nm]"),
        ],
        parsed_arguments.output_dir / "waterjet_allocator_residuals.png",
    )


if __name__ == "__main__":
    main()
