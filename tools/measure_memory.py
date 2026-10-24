"""Measure equivalent wide/packed controller state with the ARM compiler.

The starter has no robot implementation to measure as a functional baseline.
Generate the widened reference from the implemented fields and run the identical
behavioral tests against both variants before reporting the comparison.
"""
from pathlib import Path
import json
import os
import re
import shutil
import subprocess
import tempfile

from run_tests import ROOT, build, run

TYPES = [("subsystem1/telemetry.h", "telemetry_t"),
         ("subsystem2/motion.h", "motion_t"),
         ("subsystem3/line.h", "line_t"),
         ("subsystem4/safety.h", "safety_t"),
         ("subsystem4/terrain.h", "terrain_t"),
         ("subsystem5/navigation.h", "navigation_t"),
         ("subsystem5/obstacle.h", "obstacle_t")]


def main():
    with tempfile.TemporaryDirectory(prefix="motor-memory-") as temporary:
        temporary = Path(temporary)
        wide = temporary / "wide"
        shutil.copytree(ROOT / "app_program", wide / "app_program")
        shutil.copytree(ROOT / "testing/host", wide / "testing/host")
        for path in (wide / "app_program").rglob("*.h"):
            path.write_text(re.sub(r'\b(?:uint(?:8|16|32)_t|bool)\s+(\w+)\s*:\s*\d+\s*;',
                                   r'uint32_t \1;', path.read_text()))
        for name, root in [("packed", ROOT), ("wide", wide)]:
            executable = temporary / name
            if name == "wide":
                executable = temporary / "verify-wide"
            build(root, executable)
            print(name + ": " + run(executable).strip())
        probe = []
        for index, (header, typename) in enumerate(TYPES):
            probe.append(f'#include "{ROOT / "app_program" / header}"')
            original = (ROOT / "app_program" / header).read_text()
            definition = re.search(r'typedef struct\s*\{.*?\}\s*' + typename + ';',
                                   original, re.S)[0]
            definition = re.sub(r'\b(?:uint(?:8|16|32)_t|bool)\s+(\w+)\s*:\s*\d+\s*;',
                                r'uint32_t \1;', definition)
            definition = definition.replace(typename, f'memory_wide_{index}_t')
            probe += [definition, f'{typename} g_packed_{index};',
                      f'memory_wide_{index}_t g_wide_{index};']
        source = temporary / "memory_probe.c"
        source.write_text("\n".join(probe))
        obj = temporary / "memory_probe.o"
        subprocess.run(["arm-none-eabi-gcc", "-std=c99", "-mcpu=cortex-m0plus",
                        "-mthumb", "-c", str(source), "-o", str(obj)], check=True)
        symbols = subprocess.check_output(["arm-none-eabi-nm", "-S", str(obj)],
                                          text=True)
        sizes = {}
        for line in symbols.splitlines():
            columns = line.split()
            if len(columns) == 4:
                sizes[columns[3]] = int(columns[1], 16)
        rows = []
        for index, (_, typename) in enumerate(TYPES):
            rows.append({"type": typename, "wide_bytes": sizes[f'g_wide_{index}'],
                         "packed_bytes": sizes[f'g_packed_{index}']})
        result = {"compiler": subprocess.check_output(
            ["arm-none-eabi-gcc", "--version"], text=True).splitlines()[0],
            "method": "same fields; baseline uses one uint32_t per logical field",
            "rows": rows, "wide_total": sum(row["wide_bytes"] for row in rows),
            "packed_total": sum(row["packed_bytes"] for row in rows)}
        output = ROOT / "testing/evidence/memory_comparison.json"
        output.write_text(json.dumps(result, indent=2) + "\n")
        markdown = ["# Measured representation comparison", "",
                    "The ARM compiler measures one instance of each real controller",
                    "context. The reference widens each logical bit-field to uint32_t;",
                    "both variants pass the identical instrumented behavioral suite.",
                    "This is a reproducible representation baseline, not a historical",
                    "pre-change robot firmware measurement (the repository held examples).",
                    "", "| Context | Wide bytes | Packed bytes | Saved bytes |",
                    "|---|---:|---:|---:|"]
        for row in rows:
            markdown.append(f'| {row["type"]} | {row["wide_bytes"]} | '
                            f'{row["packed_bytes"]} | '
                            f'{row["wide_bytes"] - row["packed_bytes"]} |')
        markdown += [f'| Total | {result["wide_total"]} | {result["packed_total"]} | '
                     f'{result["wide_total"] - result["packed_total"]} |', "",
                     "Reproduce with `python3 tools/measure_memory.py`.", "",
                     "Allocated RTOS stacks and kernel queue headers are separate from",
                     "these task-local contexts; do not add their savings to the static",
                     "ELF BSS figure. Whole-image sizes are recorded in validation.md.", ""]
        (ROOT / "docs/memory_comparison.md").write_text("\n".join(markdown))
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
