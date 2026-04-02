"""
Controller Mapping Overlay Script
==================================
Draws text labels on a PS4-style controller diagram (2550x3300 px at 300 DPI).

The PDF contains two controllers:
  - PRIMARY   (top half of image)
  - SECONDARY (bottom half of image)

Each mapping entry defines:
  - pos   : (x, y) -- the CENTER of the rendered text label, in image pixels
  - align : 'left' | 'right' | 'center'
            left   -> text starts at pos and grows right
            right  -> text ends at pos and grows left
            center -> text is centered on pos (default for all entries here)

Coordinates were calibrated against the 2550x3300 px rasterisation of controllers.pdf.
To use a different resolution, scale all pos values by (new_width / 2550).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import TYPE_CHECKING

from PIL import Image, ImageDraw, ImageFont

if TYPE_CHECKING:
    pass

# ---------------------------------------------------------------------------
# PRIMARY MAPPINGS
# Edit the label strings to set your button names.
# Do NOT change the pos values unless you re-calibrate.
# ---------------------------------------------------------------------------

PRIMARY_MAPPINGS: dict[str, dict] = {
    # -- Shoulder / trigger buttons ------------------------------------------
    "L2": {"pos": (627, 114), "align": "center"},
    "L1": {"pos": (441, 377), "align": "center"},
    "Touchpad": {"pos": (1270, 743), "align": "center"},
    "Share": {"pos": (1113, 394), "align": "center"},
    "Options": {"pos": (1389, 241), "align": "center"},
    "R1": {"pos": (2256, 318), "align": "center"},
    "R2": {"pos": (1827, 63), "align": "center"},
    # -- D-Pad ---------------------------------------------------------------
    "D-Up": {"pos": (284, 598), "align": "center"},
    "D-Left": {"pos": (284, 815), "align": "center"},
    "D-Down": {"pos": (284, 1015), "align": "center"},
    "D-DownRight": {"pos": (284, 1236), "align": "center"},
    # -- Face buttons --------------------------------------------------------
    "Triangle": {"pos": (2277, 607), "align": "center"},
    "Circle": {"pos": (2260, 798), "align": "center"},
    "Cross": {"pos": (2256, 1002), "align": "center"},
    "Square": {"pos": (2269, 1244), "align": "center"},
    # -- Analog sticks -------------------------------------------------------
    "Left Stick": {"pos": (944, 1512), "align": "center"},
    "Right Stick": {"pos": (1607, 1542), "align": "center"},
}

SECONDARY_MAPPINGS: dict[str, dict] = {
    # -- Shoulder / trigger buttons ------------------------------------------
    "L2": {"pos": (604, 1754), "align": "center"},
    "L1": {"pos": (554, 2057), "align": "center"},
    "Touchpad": {"pos": (1258, 2464), "align": "center"},
    "Share": {"pos": (1121, 2083), "align": "center"},
    "Options": {"pos": (1389, 1911), "align": "center"},
    "R1": {"pos": (2223, 2002), "align": "center"},
    "R2": {"pos": (1921, 1760), "align": "center"},
    # -- D-Pad ---------------------------------------------------------------
    "D-Up": {"pos": (278, 2317), "align": "center"},
    "D-Left": {"pos": (278, 2521), "align": "center"},
    "D-Down": {"pos": (282, 2768), "align": "center"},
    "D-DownRight": {"pos": (282, 3023), "align": "center"},
    # -- Face buttons --------------------------------------------------------
    "Triangle": {"pos": (2319, 2307), "align": "center"},
    "Circle": {"pos": (2315, 2525), "align": "center"},
    "Cross": {"pos": (2310, 2747), "align": "center"},
    "Square": {"pos": (2323, 3016), "align": "center"},
    # -- Analog sticks -------------------------------------------------------
    "Left Stick": {"pos": (971, 3203), "align": "center"},
    "Right Stick": {"pos": (1532, 3211), "align": "center"},
}

# ---------------------------------------------------------------------------
# STYLE  -- tweak these freely
# ---------------------------------------------------------------------------

TEXT_COLOR = (20, 20, 20)  # near-black text
BG_COLOR = (255, 255, 255, 210)  # semi-transparent white pill background
FONT_SIZE = 42  # px -- sized for the 2550-wide image
PADDING = (14, 7)  # (horizontal, vertical) pill padding in px

# Tolerance for floating-point scale comparison
_SCALE_EPSILON = 1e-3

# ---------------------------------------------------------------------------
# INTERNALS
# ---------------------------------------------------------------------------


def load_font(size: int) -> ImageFont.FreeTypeFont:
    """Load a clean bold sans-serif font.

    Searches Linux paths first, then Windows, then macOS.
    Raises RuntimeError if no TrueType font is found — place any .ttf named
    ``font.ttf`` next to this script as a last-resort fallback.
    """
    candidates = [
        # Linux
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf",
        # Windows
        "C:/Windows/Fonts/arialbd.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibrib.ttf",
        "C:/Windows/Fonts/calibri.ttf",
        "C:/Windows/Fonts/verdanab.ttf",
        "C:/Windows/Fonts/verdana.ttf",
        "C:/Windows/Fonts/tahomabd.ttf",
        "C:/Windows/Fonts/tahoma.ttf",
        # macOS
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/Library/Fonts/Arial Bold.ttf",
        # Same directory as this script
        str(Path(__file__).resolve().parent / "font.ttf"),
    ]
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    raise RuntimeError(
        "No TrueType font found. Place any .ttf file named 'font.ttf' "
        "next to controller_mapper.py, or install one system-wide."
    )


def draw_label(
    draw: ImageDraw.ImageDraw,
    text: str,
    pos: tuple[int, int],
    align: str,
    font: ImageFont.FreeTypeFont,
) -> None:
    """Render a pill-shaped label so that its visual center sits at *pos*.

    *align* controls which side of *pos* the text grows toward:
      ``'center'`` -- text box centered on pos
      ``'left'``   -- text starts at pos, grows right
      ``'right'``  -- text ends at pos, grows left
    """
    cx, cy = pos
    bbox = font.getbbox(text)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    px, py = PADDING

    if align == "left":
        tx = cx
    elif align == "right":
        tx = cx - tw
    else:
        tx = cx - tw // 2

    ty = cy - th // 2

    draw.rounded_rectangle(
        [tx - px, ty - py, tx + tw + px, ty + th + py],
        radius=8,
        fill=BG_COLOR,
    )
    draw.text((tx - bbox[0], ty - bbox[1]), text, font=font, fill=TEXT_COLOR)


def apply_mappings(
    image: Image.Image,
    mappings: dict[str, dict],
    font: ImageFont.FreeTypeFont,
) -> Image.Image:
    """Composite all labels onto *image* and return the result."""
    img = image.convert("RGBA")
    overlay = Image.new("RGBA", img.size, (255, 255, 255, 0))
    draw = ImageDraw.Draw(overlay)

    for label, cfg in mappings.items():
        draw_label(draw, label, cfg["pos"], cfg["align"], font)

    return Image.alpha_composite(img, overlay).convert("RGB")


def scale_mappings(mappings: dict[str, dict], scale: float) -> dict[str, dict]:
    """Return a copy of *mappings* with all pos values scaled."""
    return {
        label: {
            "pos": (round(cfg["pos"][0] * scale), round(cfg["pos"][1] * scale)),
            "align": cfg["align"],
        }
        for label, cfg in mappings.items()
    }


# ---------------------------------------------------------------------------
# PUBLIC API
# ---------------------------------------------------------------------------


def annotate(
    input_path: str | Path,
    output_path: str | Path,
    primary: dict[str, dict] | None = None,
    secondary: dict[str, dict] | None = None,
    font_size: int = FONT_SIZE,
) -> None:
    """Overlay controller labels onto an image and save the result.

    Parameters
    ----------
    input_path:
        Path to the rasterised controller image (PNG/JPG).
    output_path:
        Where to save the annotated output.
    primary:
        Override PRIMARY_MAPPINGS (dict of ``{label: {pos, align}}``).
    secondary:
        Override SECONDARY_MAPPINGS.
    font_size:
        Font size in pixels (default sized for 2550-wide image).

    Image width is detected automatically; coordinates are scaled if it
    differs from the calibration width of 2550 px.
    """
    img = Image.open(input_path)
    scale = img.width / 2550.0
    font = load_font(font_size)

    p = primary if primary is not None else PRIMARY_MAPPINGS
    s = secondary if secondary is not None else SECONDARY_MAPPINGS

    if abs(scale - 1.0) > _SCALE_EPSILON:
        p = scale_mappings(p, scale)
        s = scale_mappings(s, scale)

    for m in [p, s]:
        img = apply_mappings(img, m, font)

    img.save(output_path, quality=95)
    print(f"Saved -> {output_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Overlay controller button labels on the diagram."
    )
    parser.add_argument("input", help="Input image (rasterised PDF page, PNG, JPG)")
    parser.add_argument("output", help="Output image path")
    parser.add_argument(
        "--font-size",
        type=int,
        default=FONT_SIZE,
        help=f"Font size in px (default: {FONT_SIZE}, calibrated for 2550-wide image)",
    )
    parser.add_argument(
        "--primary-only", action="store_true", help="Only draw PRIMARY labels"
    )
    parser.add_argument(
        "--secondary-only", action="store_true", help="Only draw SECONDARY labels"
    )
    args = parser.parse_args()

    img = Image.open(args.input)
    scale = img.width / 2550.0
    font = load_font(args.font_size)

    p = (
        scale_mappings(PRIMARY_MAPPINGS, scale)
        if abs(scale - 1.0) > _SCALE_EPSILON
        else PRIMARY_MAPPINGS
    )
    s = (
        scale_mappings(SECONDARY_MAPPINGS, scale)
        if abs(scale - 1.0) > _SCALE_EPSILON
        else SECONDARY_MAPPINGS
    )

    if args.secondary_only:
        to_apply = [s]
    elif args.primary_only:
        to_apply = [p]
    else:
        to_apply = [p, s]

    for m in to_apply:
        img = apply_mappings(img, m, font)

    img.save(args.output, quality=95)
    print(f"Saved -> {args.output}")


if __name__ == "__main__":
    main()
