#!/usr/bin/env python3
"""Post-build skin step (vst.json "skin_post"): run as `python3 skin_post.py <skin dir>`
after gen_vst has generated the skin.

Two jobs, both about the hand-edited artwork in artwork/:

1. Drop every `sh_*.png` there over the file the skin builder generated with the
   same name. Those are the pages (sh_bg_N), the keycaps, the SOURCE segments and
   the LISTEN LED -- edited outside the build, so the build must not keep its own
   version. Only files the build actually produced are replaced, so an extra file
   in artwork/ can never ship as an orphan.

2. Re-cut the stepper arrows. The builder crops each arrow tap-zone's image out of
   the page background it just drew (`shadow_skin.py`, "Stepper arrow tap-zones"),
   so those crops are stale the moment the background is replaced: the live arrow
   would show the old colours as an opaque patch over the new page. Re-cut them
   from the replacement backgrounds, using the tap-zone bounds TUI.json already
   records.

A non-zero exit fails the build (gen_vst runs this with check=True).
"""
import json
import os
import shutil
import sys

from PIL import Image


def main(skin):
    here = os.path.dirname(os.path.abspath(__file__))
    art = os.path.join(here, "artwork")
    out = os.path.join(skin, "Plugin Skins")
    if not os.path.isdir(out):
        raise SystemExit("skin_post: no Plugin Skins folder in %s" % skin)

    replaced = 0
    for name in sorted(os.listdir(art)) if os.path.isdir(art) else []:
        if not name.startswith("sh_") or not name.endswith(".png"):
            continue
        dst = os.path.join(out, name)
        if os.path.isfile(dst):               # only files this build produced
            shutil.copyfile(os.path.join(art, name), dst)
            replaced += 1

    # 2. re-cut the stepper arrows out of the (possibly new) backgrounds
    tui_path = os.path.join(out, "TUI.json")
    tui = json.load(open(tui_path))["pageData"]
    defs = {d["key"]: d["value"] for d in tui["componentDefinitions"]["localComponentDefinitions"]}
    recut = 0
    for i, tab in enumerate(tui["tabs"]):
        bg_path = os.path.join(out, "sh_bg_%d.png" % i)
        if not os.path.isfile(bg_path):
            continue
        bg = Image.open(bg_path).convert("RGB")
        page = tab["componentDefinition"] if "componentDefinition" in tab else defs[tab["componentName"]]
        for c in page["componentsData"]:
            # a stepper's arrow is a tap-zone component whose type names a local
            # definition; the Button with the cropped image lives inside that
            inner = defs.get(c["componentData"]["type"], {}).get("componentsData", [])
            for d in inner:
                if d["componentData"]["type"] != "Button":
                    continue
                img = d["componentData"]["data"].get("offImage") or ""
                if not img.startswith("sh_arrow_"):
                    continue
                x, y, w, h = [int(float(v)) for v in c["bounds"]["bounds"].split()]
                cut = bg.crop((x, y, x + w, y + h))
                cut.save(os.path.join(out, img))
                on = d["componentData"]["data"].get("onImage")
                if on and on != img:
                    cut.save(os.path.join(out, on))
                recut += 1

    print("skin_post: %d artwork file%s, %d stepper arrow%s" % (
        replaced, "" if replaced == 1 else "s", recut, "" if recut == 1 else "s"))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: skin_post.py <skin dir>")
    main(sys.argv[1])
