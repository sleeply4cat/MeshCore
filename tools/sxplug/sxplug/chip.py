"""What is known about the SX126x data space, as data (used by sim, lint and docs).

Everything here was measured on an SX1262 (Heltec MeshPocket, ROM "SX1261 V2D 2D02")
unless the comment says otherwise. See docs/hardware.md for the evidence.
"""

ROM_VERSION = b'SX1261 V2D 2D02'

# ---------------------------------------------------------------- program memory
PRAM_BASE = 0x8000
PRAM_WORDS = 2048            # 0x8000-0x87FF hold 24-bit data (written/read back)
PRAM_EXEC_CHECKED = 0x81B4   # code executed from PRAM up to here on hw (0x8000-0x81B3)

# ROM command table (index = opcode & 0x3F) -> PRAM entry. Only opcodes with bit 7
# set reach the CPU; bit 6 is ignored by the ROM dispatcher (0x9C == 0xDC).
SLOTS = {
    0x8006: (0x19, 'PRAM update / init', (0xD9,)),          # 0xD9 verified; 0x99 same index, untested
    0x800C: (0x1A, 'slot A (Semtech: LBT)', (0x9A, 0xDA)),
    0x8012: (0x1B, 'slot B (Semtech: spectral scan)', (0x9B, 0xDB)),
    0x8018: (0x1C, 'slot C (Semtech: free, returns error)', (0x9C, 0xDC)),
}
OPCODE_SLOT = {op: e for e, (_, _, ops) in SLOTS.items() for op in ops}
OPCODE_SLOT[0x99] = 0x8006
# 0x8000-0x8004 are entry points no known command calls; keep them Semtech-compatible.

# ---------------------------------------------------------------- data space
PARAMS = 0x009A              # parameter bytes of the last command (CPU commands)
PARAMS_SAFE = 0x00C0         # 0x00C0+ holds ROM variables (0x00C0, 0x00C1, 0x00CA.., 0x00DA)
VERSION_STRING = 0x0320      # 16 bytes, plain RAM
VERSION_TAG = 0x032E         # last version char; Semtech's init writes '6' here
STACK_TOP = 0x0320           # i3 software stack grows down from here
SLOT_ENTRY_I3 = 0x031C       # i3 when a slot entry (0x8006..0x8018) is reached
PLUGIN_RAM = 0x0330          # 0x0330-0x03FF: free, except 0x0380-0x0383 (written by Calibrate 0x7F) [hw]
PLUGIN_RAM_END = 0x0400
RADIO_BUF = 0x0400           # 256-byte radio data buffer (Read/WriteBuffer)
RADIO_BUF_END = 0x0500

REGIONS = [  # (start, end, name, note)
    (0x0000, 0x0080, 'sfr', 'short-address page; 0x10-0x7F alias registers 0x0580-0x090F'),
    (0x0080, 0x009A, 'rom-vars', 'initialised from ROM at reset'),
    (0x009A, 0x00C0, 'params', 'command parameter buffer'),
    (0x00C0, 0x0100, 'rom-vars', 'ROM variables (rx counter 0x00DA, ...)'),
    (0x0100, 0x0200, 'hole', 'reads 0, writes ignored'),
    (0x0200, 0x02B0, 'rom-vars', 'ROM variables: 0x0240-0x0293 SetRxDutyCycle context, 0x02A8 rx length, '
                                 '0x02A9 rx pointer, 0x02AE-0x02AF must keep their reset values 00 01 [hw]'),
    (0x02B0, 0x0320, 'stack', 'i3 stack, grows down from 0x0320; the ROM reaches 0x02E4 (CalibrateImage) [hw]'),
    (0x0320, 0x0330, 'version', 'version string'),
    (0x0330, 0x0380, 'plugin-ram', 'free for plugins (observed, not guaranteed)'),
    (0x0380, 0x0384, 'plugin-ram-cal', 'overwritten by Calibrate(0x7F) [hw]: scratch only'),
    (0x0384, 0x0400, 'plugin-ram', 'free for plugins (observed, not guaranteed)'),
    (0x0400, 0x0500, 'radio-buf', 'radio data buffer'),
    (0x0500, 0x0A00, 'registers', 'sparse hardware registers'),
]


REGIONS_STACK_FLOOR = 0x02B0    # 0x02A8-0x02AF are ROM variables right below the usable stack


def region(addr):
    for a, b, name, _ in REGIONS:
        if a <= addr < b:
            return name
    return 'none'


def _rule(addr):
    x, y = addr >> 4, addr & 0xF
    if x == 1:
        return 0x0580 + y if y < 8 else 0x0940 + (y - 8)
    return {2: 0x05A0, 3: 0x0680, 4: 0x0700, 5: 0x0800, 6: 0x0880, 7: 0x0900}[x] + y


# Short addresses whose alias was seen on hw (write one, read the other):
SFR_ALIAS_SEEN = ({0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x17, 0x18, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
                   0x21, 0x22, 0x24, 0x25, 0x26, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46,
                   0x50, 0x51, 0x54, 0x55, 0x56, 0x57, 0x59, 0x5A, 0x5B, 0x6A, 0x6B, 0x6C, 0x6D,
                   0x6E, 0x6F, 0x70, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x7D, 0x7E, 0x7F}
                  | set(range(0x30, 0x40)) | set(range(0x4A, 0x50)) | set(range(0x60, 0x69)))
# ... and ones where the reset snapshot contradicts the rule (SFR != register):
SFR_ALIAS_CONTRADICTED = {0x23, 0x29, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F}


def sfr_alias(addr, seen_only=False):
    """Short address 0x10-0x7F -> the register it aliases (same storage), else None.

    The rule (block per high nibble) holds for every address tested, but not every
    address was tested; seen_only=True restricts to observed pairs."""
    if not 0x10 <= addr <= 0x7F or addr in SFR_ALIAS_CONTRADICTED:
        return None
    if seen_only and addr not in SFR_ALIAS_SEEN:
        return None
    return _rule(addr)


def canonical(addr):
    """Physical storage address for a data-space byte (SFR aliases folded)."""
    a = sfr_alias(addr)
    return addr if a is None else a


# 0x0800/0x0801 have read-only mirrors at 0x0802/0x0803 (and SFR 0x52/0x53)
RO_MIRROR = {0x0802: 0x0800, 0x0803: 0x0801}

# ---------------------------------------------------------------- status
# The handler's r2 becomes the command-status field of GetStatus: field = r2 >> 1.
ST_OK, ST_DATA, ST_TIMEOUT, ST_ERROR, ST_FAIL, ST_TXDONE = 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C
CMD_STATUS = {1: 'ok', 2: 'data available', 3: 'timeout', 4: 'command error',
              5: 'failure to execute', 6: 'tx done'}
CHIP_MODE = {2: 'STBY_RC', 3: 'STBY_XOSC', 4: 'FS', 5: 'RX', 6: 'TX'}


def describe_status(s):
    return '0x%02X %s / %s' % (s, CHIP_MODE.get(s >> 4 & 7, 'mode?%d' % (s >> 4 & 7)),
                               CMD_STATUS.get(s >> 1 & 7, 'cmd?%d' % (s >> 1 & 7)))


# ---------------------------------------------------------------- hazards
# Writes that wedge the chip until NRESET (found by per-bit scans on hw). Rules are
# on the physical register (SFR aliases have the same effect).
# Each rule: (address, predicate(value) -> True if the value is dangerous, text)
HAZARDS = [
    (0x05D4, lambda v: True, 'any write wedges the chip'),
    (0x0900, lambda v: v & 0x30 != 0x30, 'bits 4-5 must stay set (reset 0x30)'),
    (0x0921, lambda v: not v & 0x04, 'bit 2 must stay set'),
    (0x090D, lambda v: v & 0x08, 'bit 3 must stay clear'),
    (0x05A0, lambda v: v & 0x02, 'bit 1 must stay clear'),
    (0x0951, lambda v: v & 0x01, 'bit 0 must stay clear (BUSY hangs)'),
    (0x0581, lambda v: v & 0x01, 'bit 0 is the BUSY line: setting it holds BUSY high (from the host: hangs)'),
    (0x0587, lambda v: v & 0x04 and not v & 0x0B, 'bit 2 needs one of bits 0/1/3'),
    (0x091C, lambda v: v & 0x01, 'bit 0 must stay clear'),
    (0x091D, lambda v: v & 0x40, 'bit 6 must stay clear'),
    (0x0919, lambda v: True, 'unstable: writes sometimes wedge the chip'),
    (0x02AE, lambda v: v != 0x00, 'ROM variable (reset 00): another value hangs the next GetStatus'),
    (0x02AF, lambda v: v != 0x01, 'ROM variable (reset 01): another value hangs the next GetStatus'),
]
# Writes that are not fatal but change the radio state behind the ROM's back.
RISKY = {
    0x05A1: 'chip status/mode byte (SFR 0x21); writing 0x5x starts TX',
    0x0610: 'PRAM/ROM window enable',
}


def check_write(addr, value):
    """-> (level, text) for a write to data space, or None. level: 'error' | 'warning'."""
    a = canonical(addr)
    for ha, bad, text in HAZARDS:
        if a == ha and (value is None or bad(value)):
            what = 'unknown value' if value is None else '0x%02X' % value
            return 'error', 'write %s to 0x%04X%s: %s' % (
                what, a, ' (via SFR 0x%02X)' % addr if a != addr else '', text)
    if a in RISKY:
        return 'warning', 'write to 0x%04X: %s' % (a, RISKY[a])
    return None
