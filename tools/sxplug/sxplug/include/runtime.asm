; runtime.asm -- PRAM entry table and default handlers (prepended by `sxplug build`).
;
; The ROM jumps into PRAM through fixed addresses, so the first 30 words have a
; fixed layout. It reproduces Semtech's patch word for word apart from where the
; called functions live: 0x8000-0x8004 are not called by any known command, but
; something might, so they behave exactly like Semtech's.
;
; Your program defines any of these labels (all are optional):
;   on_9a, on_9b, on_9c   command handlers for opcodes 0x9A, 0x9B, 0x9C
;                         (0xDA/0xDB/0xDC are the same slots); default: ST_ERROR
;   on_init               extra work on 0xD9 (PRAM update), after the version tag
;   SX_TAG = 'x'          version tag written to 0x032E on 0xD9 (default '6' -> "...2D06")
;
; A handler is entered with `calls` (return address in ip) and must:
;   return with `rets` (jump ip), r2 = status (ST_OK, ST_ERROR, ...), r3 = 0;
;   leave i3 as it found it; save ip (spush ip / spop ip) if it calls anything;
;   preserve i2 (Semtech's handlers save it; whether the ROM needs it is unknown).
; It may clobber a, r0-r3, i0, i1 and the flags (Semtech's handlers do).

.include "sx126x.inc"

.default on_9a, sx_cmd_error
.default on_9b, sx_cmd_error
.default on_9c, sx_cmd_error
.default on_init, _sx_nothing
.default SX_TAG, '6'

.org 0x8000
        jump _sx_e0             ; 0x8000  unknown caller
        jump _sx_e1             ; 0x8001  unknown caller
        jump _sx_e2             ; 0x8002  unknown caller
        jump _sx_e3             ; 0x8003  unknown caller
        calls _sx_tag           ; 0x8004  unknown caller (hardware-stack convention)
        ret
        spush ip                ; 0x8006  opcode 0xD9: PRAM update
        calls _sx_init
        spop ip
        rets
        spush ip                ; 0x800C  opcode 0x9A / 0xDA
        calls on_9a
        spop ip
        rets
        spush ip                ; 0x8012  opcode 0x9B / 0xDB
        calls on_9b
        spop ip
        rets
        spush ip                ; 0x8018  opcode 0x9C / 0xDC
        calls on_9c
        spop ip
        rets

_sx_e0: spush ip
        calls _sx_zero
        spop ip
        jump _sx_rets
_sx_e1: spush ip
        calls _sx_zero
        spop ip
        jump _sx_rets
_sx_e2: spush ip
        calls _sx_zero
        spop ip
        jump _sx_rets
_sx_e3: spush ip
        calls _sx_zero
        spop ip
        jump _sx_rets
_sx_rets:
        rets

_sx_tag:                        ; write the version tag (Semtech: '6')
        move i0h, #hi(VERSION_STRING)
        move i0l, #lo(VERSION_STRING)
        move a, #SX_TAG
        move (i0, VERSION_TAG - VERSION_STRING), a
        rets

_sx_zero:
        move r2, #0
        move r3, #0
        rets

_sx_init:
        spush ip
        calls _sx_tag
        calls on_init
        move r2, #ST_OK
        move r3, #0
        spop ip
        rets

sx_cmd_error:                   ; default handler: "command error"
        move r2, #ST_ERROR
        move r3, #0
_sx_nothing:
        rets

.assert _sx_e0 == 0x801E, "entry table must be 30 words"
