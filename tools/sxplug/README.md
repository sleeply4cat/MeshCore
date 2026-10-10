# sxplug assembler (vendored)

The assembler and linter of the sxplug SDK for SX126x PRAM plugins -- just what
`build-sx126x-plugins.py` needs to turn `src/helpers/radiolib/sx126x_plugins/*.asm` into
C headers. Pure Python >= 3.8, no third-party packages.

Kept: `asm.py`, `isa.py`, `image.py`, `lint.py`, `chip.py` (memory map and hazards for the
linter), `include/` (`runtime.asm`, `sx126x.inc`). The rest of the SDK -- core model,
simulator, bench host, CLI, docs, tests -- lives in the sxplug project; use it to debug a plugin
(`python3 -m sxplug sim ...`) before building it here.

Snapshot: sxplug 2026-10-10. To update, copy these files from the SDK's `sxplug/` package.
