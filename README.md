# Sysex Bandit for MPC OS

A VST2 "instrument" for MPC OS that sends and receives MIDI Sysex messages: pick
or hand-edit a `.syx` message, aim `OUT PORT` at the device you want, press SEND, 
listen on one input and save what arrives.

It makes no sound. Its "instrument" output is silence; the real I/O is a pair of
ALSA sequencer ports the plugin opens itself.

## Which file

[Download the packaged plugins for Gen 1 and 2 here.](https://www.patreon.com/dreamstatic/posts/new-dawn-and-175146717)
| Device | Zip |
|---|---|
| Gen 1 (MPC Live, Live II, One, X, Key, Force) | `Sysex-Bandit-1.0.0-mpc-armv7.zip` |
| Gen 2 (the "3" models) | `Sysex-Bandit-1.0.0-mpc-aarch64.zip` |

The installer checks the architecture and refuses the wrong one. Both builds
target MPC OS 3.x.

## Install

You need root shell access (SSH) — stock MPC OS doesn't offer it; you need a
modded unit. Back up first.

```sh
scp -r Sysex-Bandit-1.0.0 root@<device-ip>:/tmp/
ssh root@<device-ip> sh /tmp/Sysex-Bandit-1.0.0/install.sh
```

The installer stops MPC (save your project first), copies the plugin folder into
`/sdcard/Synths`, backs up `MPC.settings`, registers the plugin and starts MPC
again. Running it again upgrades in place and keeps your own files. `-y` skips
the prompt, `-t <folder>` installs to another Synths folder (e.g. a card).
`uninstall.sh` reverses it.

Then add **Sysex Bandit** to a track from the plugin browser (Instrument
plugins).

## Ports: wired for you, then pickable

This is the part that surprises people coming from the desktop app:

- **You do not choose input or output ports anywhere in MPC.** The track's MIDI
  in/out selectors don't apply — the plugin doesn't use host MIDI at all. It
  opens its own ALSA client, **"Sysex Bandit"**, with two ports: **"SysEx Out"**
  and **"SysEx In"**. They show up in `aconnect -l` under the client name
  **Sysex Bandit**.
- **The sweep finds everything for you.** Every 2 seconds the plugin walks the
  sequencer and rebuilds the `OUT PORT` and `IN PORT` lists — every readable and
  writable port, its own client and the system client excluded — so there is
  never a manual `aconnect`, and a device plugged in later appears by itself.
- **You then choose what to use, and `None` is a real choice.** Both selectors
  start on **`None`**, meaning *no devices at all*: a freshly loaded plugin
  sends nothing and hears nothing until you step onto a port. Step `OUT PORT` to
  the synth you want and SEND goes **only there** — the arrows stop at the last
  port that exists and the field shows the port's own name (`ESQ-1:Midi In`).
  A port that has since vanished reads `(gone)` and refuses
  (`out port N not found`) rather than quietly reaching everybody. `IN PORT`
  works the same way: pick your synth and the plugin counts only what *it* says,
  not the whole box's SysEx traffic. Sending works regardless of LISTEN — LISTEN
  only gates receiving.

In short: install, load, step `OUT PORT` onto your device, flip **LISTEN** on
and step `IN PORT` onto your source. Nothing else to configure.

## Pages

**SEND** — `MESSAGE` picks a slot (1–16) from the plugin's `messages/` folder;
`SOURCE` switches between the slot's file and hand-edited bytes; `PREV`/`NEXT`
step through slots; `SEND` transmits; `RESCAN` re-reads the `messages/` folder
(do this after adding files yourself). `OUT PORT` (default **`None`**) aims the
send at one output device; its `◀`/`▶` step through what's connected, and with
it on `None` SEND reports `pick OUT PORT` instead of going anywhere. `TX` shows
**the message you have selected** — the slot's file or the hand-edited bytes —
as hex wrapped across all six lines of the box, exactly the way `RX` shows what
arrived; it reads `-` while `OUT PORT` is `None`. `STATUS` is the last action
("sent 6 B", errors).

**BYTES** — the hand-editor: `COUNT` (1–8) and `B1`–`B8` (0–255, decimal) are
the raw bytes; `SEND` transmits the first `COUNT` of them. The defaults are the
identity request `F0 7E 7F 06 01 F7` (240 126 127 6 1 247). Note a message sent
this way does **not** have to end in `F7` — count is count.

**RECEIVE** —
- `LISTEN` (default **OFF**): subscribes/unsubscribes SysEx In.
- `IN PORT` (default **`None`**): hear nobody until you pick a source, or pick
  one port and only it counts. This is the cure for an `RX TOTAL` that climbs
  while nothing of yours is sending — MPC OS and other apps talk SysEx too, and
  hearing everything counts all of it.
- `RX`: the last complete message received, as hex wrapped across all six lines
  of the big box — the same display `TX` uses for the selected message.
- `RX TOTAL`: **count of complete messages received, not bytes** — it wraps at
  999. Your 210-byte patch counts as 1. A long dump arriving in pieces counts
  as 1 too: ALSA cuts SysEx at 4096 bytes per event, so a **bank dump** arrives
  as a run of fragments and the plugin reassembles `F0 … F7` across them before
  counting (stray bytes between messages are ignored, and an abandoned partial
  message is dropped when a new `F0` starts). Messages up to **1 MB** are
  reassembled, counted and stored whole, so `SAVE RX` writes a bank dump back
  byte for byte.
- `STATUS`: last event and who sent it, e.g. `rx 210 B from ESQ-1` — that name
  is how you find out what has been talking.
- `SAVE RX`: writes the last received message to `messages/rx_NNN.syx`; it
  becomes a SEND slot after the automatic re-scan.
- `CLEAR`: resets `RX`, `RX TOTAL` and the saved-message buffer — start a fresh
  capture without reloading the plugin.

## Your own messages

The plugin's message folder lives inside its own folder on the device:
`/sdcard/Synths/Dream Static Sounds - VST - Sysex Bandit/messages/`. Drop any
`.syx` file in there (up to 16 slots, sorted by name), press `RESCAN`, and it
appears under `MESSAGE`. Files you add are kept across upgrades and uninstalls.

## Q-Links

Q-Links follow the page on screen. On SEND: CC 20 = MESSAGE, CC 21 = SOURCE,
CC 22 = OUT PORT. On RECEIVE: CC 20 = LISTEN, CC 21 = CLEAR, CC 22 = IN PORT.

## Troubleshooting

- **Nothing arrives**: is `LISTEN` on? Is `IN PORT` still `None`, or pointed at
  the right source? Is the sender actually transmitting? `aconnect -l` shows
  whether the subscriptions exist — look for the client **Sysex Bandit** and
  who is connected to its `SysEx In`.
- **RX TOTAL keeps climbing when you didn't send anything**: something on the
  box is talking SysEx — MPC OS and other apps do too. Point `IN PORT` at your
  device and only its traffic counts; `STATUS` names whoever is sending either
  way. Press CLEAR for a fresh count, or turn LISTEN off when you're not
  capturing.
- **SEND says `pick OUT PORT`**: `OUT PORT` is on `None`, so there is no
  destination at all. Step it onto the device you want.
- **`out port N not found`**: the picked device was unplugged. The plugin
  refuses rather than broadcasting your dump to everything else — step
  `OUT PORT` onto a port that is still there, or back to `None`.
- **No sound is expected** — it's a silent instrument; the audio output is
  always silence.
- **A send seems to vanish**: check nothing else (another app) is holding the
  receiver; the plugin can't detect a busy device.

## For developers

The port lives at the root of this repository (`src/sysex_bandit.c`, `build.sh`,
`test/`) and is built with [mpc-vst-plugins](https://github.com/open-mpc/mpc-vst-plugins)
— the kit is used, not vendored. Check it out beside this repo as
`../mpc-vst-plugins`, or point `MPC_VST` at it:

```sh
MPC_VST=/path/to/mpc-vst-plugins ./build.sh              # armv7 (MM-303 armhf image)
MPC_VST=/path/to/mpc-vst-plugins ARCH=aarch64 ./build.sh # aarch64 (mm303:aarch64-audio)
./test/host_test.sh           # x86 host test, ASan/UBSan
./test/engine_loop.sh         # live ALSA loopback: sweep, both port pickers,
                              # send targeting, reassembly, LISTEN, CLEAR
```

The kit supplies the VST2 wrapper (`wrapper/vst2_wrap.c`), the params/skin
generator (`tools/gen_vst.py`), the HTML art renderer and the release
packager; this repo supplies the engine, skin sources and tests. Release zips
are produced with the kit's `tools/release.py` (see the kit's
`docs/RELEASING.md`) and land in `dist/`, which is not tracked here — publish
the zips as GitHub Release assets.

The skin uses the kit's **browser renderer** (`vst.json` `"art": "html"`), so
`build.sh` runs `gen_vst.py` inside the `mpc-vst-html-art` image instead of the
bitmap `shadow_art` — that is what buys the real Titillium Web text and the
knob/keycap sheen. The look is split four ways:

| file | holds |
|---|---|
| `layout.conf` | geometry, `theme_*` palette, the `box=0` line readouts |
| `skin.css` | font sizes, `--sheen`, the few per-class overrides |
| `artwork/*.png` | **the hand-edited pages, keycaps, segments and LED** |
| `bg.svg`, `btn.svg` | fallbacks used only if `artwork/` is missing |

`artwork/` is not part of the layout: `vst.json` names `skin_post.py`, which
gen_vst runs after the skin is built to drop those PNGs over the generated ones
and then re-cut the stepper arrows out of the new pages (the builder crops them
from the page it drew, so they go stale the moment the page is replaced).

MPC's Label does not wrap, so `TX` and `RX` are **one readout per line** — six
each, `tx2..tx6` / `rx2..rx6` — with `TX_BPL`/`RX_BPL` in `src/sysex_bandit.c`
setting how many bytes a line holds. Those two numbers and the line geometry in
`layout.conf` have to move together: a line is sized to just fill its box at
that box's `tsize`.

## Acknowledgements

Built with [mpc-vst-plugins](https://github.com/open-mpc/mpc-vst-plugins)
(MIT, © sd88me): the VST2 wrapper, the params/skin generator, the HTML art
renderer and the release packager. `test/host_test.c` is adapted from the
kit's `tools/host_test.c`.

## License

MIT — see [LICENSE](LICENSE).
