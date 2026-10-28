#!/bin/sh
# Fetches the Arduino libraries that the firmware uses, at fixed versions, into simulator/.deps.
# The simulator compiles these unmodified sources together with nutrient_mixer.ino.
set -e

DEPS_DIR="${1:-$(dirname "$0")/../.deps}"
mkdir -p "$DEPS_DIR"

fetch() {
  name="$1"
  repo="$2"
  tag="$3"
  if [ -f "$DEPS_DIR/$name/.fetched-$tag" ]; then
    return
  fi
  echo "Fetching $name $tag"
  rm -rf "$DEPS_DIR/$name"
  git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$tag" "https://github.com/$repo.git" "$DEPS_DIR/$name"
  touch "$DEPS_DIR/$name/.fetched-$tag"
}

fetch GEM Spirik/GEM 1.8.2
fetch Adafruit-GFX adafruit/Adafruit-GFX-Library 1.11.11
fetch KeyDetector Spirik/KeyDetector 1.2.1
fetch AiEsp32RotaryEncoder igorantolic/ai-esp32-rotary-encoder 1.7
fetch PH4502C-Sensor nthnn/PH4502C-Sensor v1.1.1
fetch QR-Code-generator nayuki/QR-Code-generator v1.8.0
