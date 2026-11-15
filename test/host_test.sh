#!/usr/bin/env bash
# Offline x86 run of the kit's host test against this port, before anything goes to a
# device. The kit's tools/test_port.sh builds its own tools/host_test.c (which carries
# that kit's own fx_reset tests), so this script builds the vendored test/host_test.c
# instead: generate params.h, compile engine + wrapper + test under ASan/UBSan, link the
# host's libasound, run it (exit status = the test's).
#
#   ./host_test.sh
#   MPC_VST=... ALSA_INC=... ALSA_LIB=... to override paths (defaults: the kit checkout
#   beside this repo (../mpc-vst-plugins), /tmp/alsa-dev headers, the host's runtime
#   libasound). The kit is mpc-vst-plugins: https://github.com/open-mpc/mpc-vst-plugins
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"          # repo root
MV="${MPC_VST:-$HERE/../mpc-vst-plugins}"
[ -f "$MV/tools/gen_vst.py" ] || {
  echo "host_test.sh: mpc-vst-plugins not found at '$MV'" >&2
  echo "set MPC_VST=<path-to-kit> (upstream: https://github.com/open-mpc/mpc-vst-plugins)" >&2
  exit 1; }
ALSA_INC="${ALSA_INC:-/tmp/alsa-dev/usr/include}"
ALSA_LIB="${ALSA_LIB:-/usr/lib/x86_64-linux-gnu/libasound.so.2}"

python3 "$MV/tools/gen_vst.py" "$HERE/vst.json" --params-h
mkdir -p "$HERE/build"
# the engine looks for messages beside the binary (MODULE_SUBDIR), as it does on a device
rm -rf "$HERE/build/messages"
cp -r "$HERE/messages" "$HERE/build/messages"

SAN="-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1"
OBJS=()
for f in src/sysex_bandit.c "$MV/wrapper/vst2_wrap.c" test/host_test.c; do
  case "$f" in /*) src="$f" ;; *) src="$HERE/$f" ;; esac
  o="$HERE/build/ht_$(basename "$f" .c).o"
  gcc $SAN -std=gnu11 -I"$HERE/build" -I"$MV/wrapper" -I"$ALSA_INC" -c "$src" -o "$o"
  OBJS+=("$o")
done
gcc $SAN "${OBJS[@]}" "$ALSA_LIB" -lpthread -lm -o "$HERE/build/host_test"
"$HERE/build/host_test"
