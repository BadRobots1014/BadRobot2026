"""
generate_controller_map.py
==========================
Regenerates controller diagram images only when bindings have changed.

This script has NO dependency on RobotPy, commands2, wpilib, or ntcore.
It reads controller_bindings.json, which CustomController writes automatically
every time a button is bound during a normal robot run or sim session.

Workflow
--------
1. Run / sim your robot once — controller_bindings.json is written automatically.
2. Run this script before deploying:
       python generate_controller_map.py
3. Commit controller_bindings.json so CI / teammates get fresh images too.

File layout (relative to this script)
--------------------------------------
  ./ (e.g. robot/controllers/)
    custom_controller.py
    mapper.py
    generate_controller_map.py
  ../
    controller_bindings.json        <- written by CustomController at runtime
  ../deploy/
    controller_map.jpg              <- generated output
    controller_map.jpg.hash         <- hash cache (commit this)
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve().parent

SOURCE_IMAGE = _HERE / "controller_diagram.jpg"
BINDINGS_FILE = _HERE / ".." / "controller_bindings.json"
OUTPUT_DIR = _HERE

FONT_SIZE = 42  # px, calibrated for 2550-wide source image

# Maps CustomController button name strings -> PRIMARY_MAPPINGS / SECONDARY_MAPPINGS keys
_BUTTON_TO_MAPPING_KEY: dict[str, tuple[str, str]] = {
    "SQUARE": ("Square", "Square"),
    "CROSS": ("Cross", "Cross"),
    "CIRCLE": ("Circle", "Circle"),
    "TRIANGLE": ("Triangle", "Triangle"),
    "L1": ("L1", "L1"),
    "R1": ("R1", "R1"),
    "L2": ("L2", "L2"),
    "R2": ("R2", "R2"),
    "SHARE": ("Share", "Share"),
    "OPTION": ("Options", "Options"),
    "TRACKPAD": ("Touchpad", "Touchpad"),
    "POVUP": ("D-Up", "D-Up"),
    "POVDOWN": ("D-Down", "D-Down"),
    "POVLEFT": ("D-Left", "D-Left"),
    "POVRIGHT": ("D-DownRight", "D-DownRight"),
    # L3/R3/HOME/UNDEFINED have no arrow in the diagram — intentionally omitted
}

# ---------------------------------------------------------------------------
# HELPERS
# ---------------------------------------------------------------------------


def _load_bindings(bindings_file: Path) -> dict[str, dict[str, str]]:
    """Load controller_bindings.json -> ``{port_str: {button: command}}``."""
    try:
        return json.loads(bindings_file.read_text(encoding="utf-8"))
    except FileNotFoundError as err:
        raise FileNotFoundError(
            f"Bindings file not found: {bindings_file}\n"
            "Run or sim your robot at least once so CustomController can write it."
        ) from err


def _primary_only(port_bindings: dict[str, str]) -> dict[str, dict]:
    """Build a primary-half mappings dict from one port's bindings.

    Only buttons that have been explicitly bound appear in the result —
    unbound buttons produce no label on the diagram.
    """
    from mapper import PRIMARY_MAPPINGS  # noqa: PLC0415

    result: dict[str, dict] = {}
    for button_name, command_name in port_bindings.items():
        keys = _BUTTON_TO_MAPPING_KEY.get(button_name)
        if keys is None:
            continue
        p_key, _ = keys
        if p_key in PRIMARY_MAPPINGS:
            result[command_name] = PRIMARY_MAPPINGS[p_key]
    return result


def _secondary_only(port_bindings: dict[str, str]) -> dict[str, dict]:
    """Build a secondary-half mappings dict from one port's bindings."""
    from mapper import SECONDARY_MAPPINGS  # noqa: PLC0415

    result: dict[str, dict] = {}
    for button_name, command_name in port_bindings.items():
        keys = _BUTTON_TO_MAPPING_KEY.get(button_name)
        if keys is None:
            continue
        _, s_key = keys
        if s_key in SECONDARY_MAPPINGS:
            result[command_name] = SECONDARY_MAPPINGS[s_key]
    return result


def _mappings_hash(
    primary: dict[str, dict],
    secondary: dict[str, dict],
    source_path: Path,
) -> str:
    payload = {
        "source": str(source_path),
        "primary": dict(sorted(primary.items())),
        "secondary": dict(sorted(secondary.items())),
    }
    raw = json.dumps(payload, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _hash_file(output_dir: Path) -> Path:
    return output_dir / "controller_map.jpg.hash"


def _output_file(output_dir: Path) -> Path:
    return output_dir / "controller_map.jpg"


def _read_hash(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return ""


def _write_hash(path: Path, digest: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(digest, encoding="utf-8")


# ---------------------------------------------------------------------------
# PUBLIC API
# ---------------------------------------------------------------------------


def maybe_regenerate(
    source_image: Path = SOURCE_IMAGE,
    bindings_file: Path = BINDINGS_FILE,
    output_dir: Path = OUTPUT_DIR,
    font_size: int = FONT_SIZE,
    force: bool = False,
    verbose: bool = True,
) -> list[Path]:
    """Regenerate the combined controller map image if bindings have changed.

    Port 0 bindings are drawn on the top half (PRIMARY / first controller).
    Port 1 bindings are drawn on the bottom half (SECONDARY / second controller).

    Returns a list containing the output path if it was (re)generated, else ``[]``.
    """
    sys.path.insert(0, str(_HERE))
    from mapper import annotate  # noqa: PLC0415

    all_bindings = _load_bindings(bindings_file)

    if not all_bindings:
        if verbose:
            print("[controller_map] controller_bindings.json is empty — nothing to do.")
        return []

    primary = _primary_only(all_bindings.get("0", {}))
    secondary = _secondary_only(all_bindings.get("1", {}))

    out = _output_file(output_dir)
    hf = _hash_file(output_dir)

    digest = _mappings_hash(primary, secondary, source_image)
    saved = _read_hash(hf)

    if not force and out.is_file() and digest == saved:
        if verbose:
            print(f"[controller_map] Bindings unchanged — skipping. ({out})")
        return []

    reason = "forced" if force else ("missing" if not out.is_file() else "changed")
    if verbose:
        print(f"[controller_map] Bindings {reason} — regenerating {out} ...")

    output_dir.mkdir(parents=True, exist_ok=True)
    annotate(
        source_image, out, primary=primary, secondary=secondary, font_size=font_size
    )
    _write_hash(hf, digest)

    if verbose:
        print("[controller_map] Done.")

    return [out]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    _force = "--force" in sys.argv
    maybe_regenerate(force=_force, verbose=True)
