#!/usr/bin/env bash
# Build and run test/engine_loop.c against the port on the host (live /dev/snd/seq).
# Same recipe as host_test.sh, minus the vendored host test, plus this one.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"          # repo root
MV="${MPC_VST:-$HERE/../mpc-vst-plugins}"
[ -f "$MV/tools/gen_vst.py" ] || {
  echo "engine_loop.sh: mpc-vst-plugins not found at '$MV'" >&2
  echo "set MPC_VST=<path-to-kit> (upstream: https://github.com/open-mpc/mpc-vst-plugins)" >&2
  exit 1; }
ALSA_INC="${ALSA_INC:-/tmp/alsa-dev/usr/include}"
ALSA_LIB="${ALSA_LIB:-/usr/lib/x86_64-linux-gnu/libasound.so.2}"

python3 "$MV/tools/gen_vst.py" "$HERE/vst.json" --params-h
mkdir -p "$HERE/build"
rm -rf "$HERE/build/messages"
cp -r "$HERE/messages" "$HERE/build/messages"

SAN="-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1"
OBJS=()
for f in src/sysex_bandit.c "$MV/wrapper/vst2_wrap.c" test/engine_loop.c; do
  case "$f" in /*) src="$f" ;; *) src="$HERE/$f" ;; esac
  o="$HERE/build/el_$(basename "$f" .c).o"
  gcc $SAN -std=gnu11 -I"$HERE/build" -I"$MV/wrapper" -I"$ALSA_INC" -c "$src" -o "$o"
  OBJS+=("$o")
done
gcc $SAN "${OBJS[@]}" "$ALSA_LIB" -lpthread -lm -o "$HERE/build/engine_loop"
cd "$HERE" && ./build/engine_loop
