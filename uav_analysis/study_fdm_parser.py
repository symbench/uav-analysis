#!/usr/bin/env python3
# Copyright (C) 2022, Peter Volgyesi
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Parse a parametric study (zip or folder) for FDM results."""
import csv
import sys
import zipfile
from pathlib import Path

from .tweak_fdm_input import parse_fdm_output


def process_study(folder_iterator):
    """Process a study folder."""
    results = []
    for folder in folder_iterator:
        if not folder.is_dir():
            continue
        out_filename = folder / "flightDynFastOut.out"
        if not out_filename.is_file():
            continue
        with out_filename.open() as out_file:
            fdm_result = parse_fdm_output(out_file.read())

        fdm_result["#study"] = folder.name  # make it the first column
        results.append(fdm_result)
    return results


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "data",
        help="a data.zip file or extracted folder (uam_direct2cad workflow)",
    )

    args = parser.parse_args(args)
    zip_or_folder = Path(args.data)
    study_iterator = None
    zip_file = None

    if zip_or_folder.is_dir():
        study_iterator = (zip_or_folder / "archive").iterdir()
    elif zip_or_folder.is_file() and zip_or_folder.suffix == ".zip":
        zip_file = zipfile.ZipFile(zip_or_folder)
        study_iterator = zipfile.Path(zip_file, at="archive/").iterdir()
    else:
        print(f"cannot find {args.data}")

    results = process_study(study_iterator)

    if results:
        writer = csv.DictWriter(
            sys.stdout,
            fieldnames=sorted(results[0].keys()),
            lineterminator="\n",
        )
        writer.writeheader()
        for result in results:
            writer.writerow(result)

    if zip_file is not None:
        zip_file.close()


if __name__ == "__main__":
    run()
