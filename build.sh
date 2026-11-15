#!/usr/bin/env bash
# Build Sysex Bandit for MPC OS as an MPC OS VST2 instrument (armv7 Gen1 / aarch64 Gen2).
#   ./build.sh                     armv7 (Gen 1), compiled inside the MM-303 armhf image
#   ARCH=aarch64 ./build.sh        64-bit ARM (Gen 2), inside mm303:aarch64-audio
#   MPC_VST=... to use another mpc-vst-plugins checkout
#   (upstream: https://github.com/open-mpc/mpc-vst-plugins)
#
# Both arches compile inside MM-303 build images (native GCC, glibc 2.31 so the plugin also
# loads on MPC OS 2.x): the engine needs ALSA (snd_seq) headers/libs, which those images ship.
# Steps follow the kit's tools/build_port.sh, but the containers are the MM-303 ones and the
# link adds -lasound; the wrapper and tools are never vendored here.
# Output: build/package/Dream Static Sounds - VST - Sysex Bandit/ (armhf)
#         build/package-aarch64/...                            (aarch64)
#         plus build/<so>, build/skin/..., build/params.h, build/pluginlist-entry.xml
set -euo pipefail
cd "$(dirname "$0")"
MPC_VST="${MPC_VST:-$PWD/../mpc-vst-plugins}"
[ -f "$MPC_VST/tools/gen_vst.py" ] || {
  echo "build.sh: mpc-vst-plugins not found at '$MPC_VST'" >&2
  echo "set MPC_VST=<path-to-kit> (upstream: https://github.com/open-mpc/mpc-vst-plugins)" >&2
  exit 1; }
ARCH="${ARCH:-armhf}"
MM303_ARMHF="${MM303_ARMHF:-mm303:armhf-glibc231-audio}"
MM303_AARCH64="${MM303_AARCH64:-mm303:aarch64-audio}"
NAME="Dream Static Sounds - VST - Sysex Bandit"
case "$ARCH" in
  armhf)   OUT="build/arm"     PKGDIR="build/package"     IMAGE="$MM303_ARMHF" ;;
  aarch64) OUT="build/arm64"   PKGDIR="build/package-aarch64" IMAGE="$MM303_AARCH64" ;;
  *) echo "build.sh: ARCH must be armhf or aarch64 (got '$ARCH')" >&2; exit 1 ;;
esac
eval "$(python3 "$MPC_VST/tools/gen_vst.py" vst.json --shell)"

mkdir -p build/tools "$OUT"
# skin artwork renderer: "art": "html" in vst.json (gradient plate, keycap images, real font)
# draws in the kit's browser image; otherwise the kit's vendored shadow_art host binary.
if [ "$ART" = html ]; then
  docker image inspect mpc-vst-html-art >/dev/null 2>&1 || \
    docker build -q -t mpc-vst-html-art "$MPC_VST/tools/html_art" >/dev/null
else
  [ -x build/tools/shadow_art ] || \
    gcc -O2 -w -I"$MPC_VST/tools/vendor/force-shadow/tools" -x c -o build/tools/shadow_art "$MPC_VST/tools/shadow_art.c" -lm
fi

# params.h, the skin, pluginlist-entry.xml. gen_vst prints "warning: skin:" lines for skin_check
# findings (touch boxes off the screen, overlaps, Q-Links on unseen params): this port has no
# popups, so every finding is a real one and fails the build (the kit's build_port.sh lets the
# caller gate; here there is nothing by-design to exempt).
gen_err=$(mktemp)
if [ "$ART" = html ]; then
  docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:$PWD" -v "$MPC_VST:$MPC_VST:ro" \
    -w "$PWD" mpc-vst-html-art python3 "$MPC_VST/tools/gen_vst.py" vst.json 2>"$gen_err" || {
    cat "$gen_err" >&2; rm -f "$gen_err"; echo "build.sh: gen_vst failed" >&2; exit 1; }
else
  SHADOW_ART="$PWD/build/tools/shadow_art" python3 "$MPC_VST/tools/gen_vst.py" vst.json 2>"$gen_err" || {
    cat "$gen_err" >&2; rm -f "$gen_err"; echo "build.sh: gen_vst failed" >&2; exit 1; }
fi
findings=$(grep "^warning: skin:" "$gen_err" || true)
if [ -n "$findings" ]; then cat "$findings" >&2; rm -f "$gen_err"; echo "build.sh: skin_check found problems" >&2; exit 1; fi
grep -v "^warning: skin:" "$gen_err" >&2 || true
rm -f "$gen_err"
[ -f build/params.h ] && [ -d "build/skin/$NAME/Plugin Skins" ]

# the plugin: engine + wrapper inside the MM-303 image for this arch
U="$(id -u):$(id -g)"
MOUNTS=(-v "$PWD:$PWD")
case "$MPC_VST" in "$PWD"/*) ;; *) MOUNTS+=(-v "$MPC_VST:$MPC_VST:ro") ;; esac
# the source list is built on the host so paths with spaces stay one word inside the container
SRCS=()
for f in $SOURCES; do SRCS+=("$PWD/$f"); done
SRCS+=("$MPC_VST/wrapper/vst2_wrap.c")
docker run --rm --platform "$([ "$ARCH" = aarch64 ] && echo linux/arm64 || echo linux/arm/v7)" \
  -u "$U" -e HOME=/tmp "${MOUNTS[@]}" -w "$PWD" "$IMAGE" bash -euc "
    OBJS=''
    for f in \"\$@\"; do
      o=\"$OUT/\$(echo \"\$f\" | tr '/ ' '__').o\"
      gcc -O2 -fPIC -fvisibility=hidden -ffunction-sections -fdata-sections -ffp-contract=off -DNDEBUG -w -std=gnu11 \
          $CFLAGS -Ibuild -I'$MPC_VST/wrapper' -c \"\$f\" -o \"\$o\"
      OBJS=\"\$OBJS \$o\"
    done
    printf '{\n  global: VSTPluginMain;\n  local: *;\n};\n' > '$OUT/exports.map'
    gcc -O2 -fPIC -shared -Wl,--no-undefined -Wl,--version-script='$OUT/exports.map' -Wl,--gc-sections -Wl,-s \
        \$OBJS $LIBS -o '$OUT/$SO'
    strip '$OUT/$SO'
    echo \"-> $OUT/$SO (\$(file -b '$OUT/$SO' | cut -d, -f1-2))\"
    echo \"exported: \$(readelf --dyn-syms -W '$OUT/$SO' | grep -E ' GLOBAL .* [0-9]+ [A-Za-z]' | grep -v UND | awk '{print \$8}' | tr '\n' ' ')\"
    echo \"highest glibc: \$(readelf -V '$OUT/$SO' | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1) (device has 2.39)\"
  " _ "${SRCS[@]}"

# package: the skin folder, the binary and the message folder the engine reads beside the .so
# (MODULE_SUBDIR "messages": the 7 shipped .syx defaults, and where save_rx writes)
PKG="$PKGDIR/$NAME"
rm -rf "$PKGDIR" && mkdir -p "$PKG"
cp -r "build/skin/$NAME/." "$PKG/"
cp "$OUT/$SO" "$PKG/"
cp -r messages "$PKG/messages"
echo "-> $PKG"
