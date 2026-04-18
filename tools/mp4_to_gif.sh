#!/bin/bash
# mp4_to_gif.sh - Convert MP4 to compact GIF for README/documentation
#
# Usage:
#   ./mp4_to_gif.sh input.mp4                  # output: input.gif (480px wide, 10fps)
#   ./mp4_to_gif.sh input.mp4 output.gif       # custom output path
#   ./mp4_to_gif.sh input.mp4 output.gif 320   # custom width (px)
#   ./mp4_to_gif.sh input.mp4 output.gif 480 15 # custom width and fps
set -e

INPUT="${1:?Usage: $0 input.mp4 [output.gif] [width] [fps]}"
OUTPUT="${2:-${INPUT%.mp4}.gif}"
WIDTH="${3:-480}"
FPS="${4:-10}"

if [ ! -f "$INPUT" ]; then
  echo "Error: $INPUT not found" >&2
  exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "Error: ffmpeg is required" >&2
  exit 1
fi

echo "Converting: $(basename "$INPUT")"
echo "  Resolution: ${WIDTH}px wide, ${FPS} fps"

# Two-pass for optimal palette (much better quality than single-pass)
PALETTE=$(mktemp /tmp/palette_XXXXXX.png)
trap "rm -f $PALETTE" EXIT

ffmpeg -y -i "$INPUT" \
  -vf "fps=${FPS},scale=${WIDTH}:-1:flags=lanczos,palettegen=stats_mode=diff" \
  "$PALETTE" 2>/dev/null

ffmpeg -y -i "$INPUT" -i "$PALETTE" \
  -lavfi "fps=${FPS},scale=${WIDTH}:-1:flags=lanczos [x]; [x][1:v] paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" \
  "$OUTPUT" 2>/dev/null

INPUT_SIZE=$(du -h "$INPUT" | cut -f1)
OUTPUT_SIZE=$(du -h "$OUTPUT" | cut -f1)
echo "  ${INPUT_SIZE} -> ${OUTPUT_SIZE}: $(basename "$OUTPUT")"
