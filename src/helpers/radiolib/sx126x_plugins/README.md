# SX126x PRAM plugins

Code that runs on the SX126x's own core (CoolRISC 816), loaded into its program RAM at run
time (`SX126x::uploadPatch`). Written in sxplug assembler; `build-sx126x-plugins.py` (a
PlatformIO pre-script of the Mesh Pocket envs) generates `X.h` from `X.asm` when the header is
missing or older than the source, and fails the build on assembler or lint errors. The headers
are build output and not committed.

- `spectrum.asm` -- a whole frequency sweep inside the radio (opcode 0x9B), used by
  `SX126xSpectrumScanner` for the spectrum screen. Verified on an SX1262, ROM "SX1261 V2D 2D02".
