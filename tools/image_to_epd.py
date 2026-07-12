#!/usr/bin/env python3
"""
Convert a PNG (or the base64 PNG embedded in a .piskel file) into a packed
1-bit C byte array, row-major, each row padded to a whole byte. A set bit is
a foreground (drawn) pixel; a clear bit is background. Two bit orders are
supported via --format:
  epd  (default) MSB-first, what EPD_ShowPicture() expects (CrowPanel).
  xbm             LSB-first, what ScreenDisplay::drawXbm() expects (Heltec).

Usage:
    python image_to_epd.py icon.png --name bookIcon
    python image_to_epd.py Book.piskel --name bookIcon
    python image_to_epd.py qr.png --name wifiQr --size 96
    python image_to_epd.py icon.piskel --name bookIcon --format xbm

For .piskel files, extracts the first layer's first frame. For any input,
if --size isn't given, uses the image's native size (rounding down to a
multiple of 8 in width isn't required -- padding is handled automatically).
--size N resizes to an NxN square first (nearest-neighbor, so QR module
edges stay crisp instead of blurring).

Output is a ready-to-paste C snippet: a `static const uint8_t NAME[] = {...}`
array plus its width/height as `#define NAME_W`/`NAME_H`.
"""
import sys
import json
import base64
import argparse
from io import BytesIO
from PIL import Image


def load_piskel_image(path):
    data = json.loads(open(path, "r", encoding="utf-8").read())
    piskel = data["piskel"]
    layer0 = json.loads(piskel["layers"][0])
    b64 = layer0["chunks"][0]["base64PNG"]
    b64 = b64.split(",", 1)[1] if b64.startswith("data:") else b64
    return Image.open(BytesIO(base64.b64decode(b64)))


def load_image(path):
    if path.lower().endswith(".piskel"):
        return load_piskel_image(path)
    return Image.open(path)


def to_1bit(img, size=None, threshold=128, invert=False):
    if img.mode in ("RGBA", "LA") or (img.mode == "P" and "transparency" in img.info):
        img = img.convert("RGBA")
        bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
        img = Image.alpha_composite(bg, img)
    img = img.convert("L")

    if size and size < img.width:
        # BOX averages each target pixel over its source region instead of
        # sampling a single source pixel -- important for QR codes, where a
        # bad single-pixel sample near a module edge can flip that module.
        img = img.resize((size, size), Image.BOX)
    elif size and size > img.width:
        # NEAREST for upscaling keeps hand-drawn pixel art crisp and blocky
        # (clean pixel-doubling/tripling) instead of blurring it.
        img = img.resize((size, size), Image.NEAREST)

    px = img.point(lambda p: 255 if p >= threshold else 0)
    bitmap = px.point(lambda p: 0 if p >= threshold else 1)  # 1 = dark/foreground
    if invert:
        bitmap = bitmap.point(lambda p: 1 - p)
    return bitmap


def pack_bits(bitmap, fmt="epd"):
    w, h = bitmap.size
    row_bytes = (w + 7) // 8
    out = bytearray(row_bytes * h)
    px = bitmap.load()
    for y in range(h):
        for x in range(w):
            if px[x, y]:
                bit = (1 << (x % 8)) if fmt == "xbm" else (0x80 >> (x % 8))
                out[y * row_bytes + (x // 8)] |= bit
    return bytes(out), w, h


def format_c_array(name, data, width, height):
    lines = [f"// {width}x{height}, {len(data)} bytes"]
    lines.append(f"#define {name.upper()}_W {width}")
    lines.append(f"#define {name.upper()}_H {height}")
    lines.append(f"static const uint8_t {name}[] = {{")
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        lines.append("  " + ", ".join(f"0x{b:02X}" for b in chunk) + ",")
    lines.append("};")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input")
    parser.add_argument("--name", required=True, help="C identifier for the array")
    parser.add_argument("--size", type=int, default=None, help="resize to NxN first (nearest-neighbor)")
    parser.add_argument("--threshold", type=int, default=128)
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("--format", choices=["epd", "xbm"], default="epd",
                         help="epd = MSB-first (EPD_ShowPicture, CrowPanel); xbm = LSB-first (drawXbm, Heltec)")
    parser.add_argument("--out", default=None, help="write to this file instead of stdout")
    args = parser.parse_args()

    img = load_image(args.input)
    bitmap = to_1bit(img, size=args.size, threshold=args.threshold, invert=args.invert)
    data, w, h = pack_bits(bitmap, fmt=args.format)
    code = format_c_array(args.name, data, w, h)

    if args.out:
        with open(args.out, "w") as f:
            f.write(code + "\n")
        print(f"Wrote {args.out} ({w}x{h}, {len(data)} bytes)")
    else:
        print(code)


if __name__ == "__main__":
    main()
