; spectrum -- a whole frequency sweep inside the radio, for an SDR-like spectrum view.
; Build with sxplug: python3 -m sxplug build spectrum.asm -o spectrum   (-> spectrum.h)
;
; The chip retunes itself by writing the frequency registers (REG_FREQ, [0x6B..0x6E], MSB first: the
; write of the LSB commits the frequency), checked on hardware against SetRfFrequency.
;
; Pipelined: the RSSI register (GFSK) updates every T ~ 1.6/RBW ms (RBW in kHz), but a frequency change
; only shows in it ~3.4 T later (~4 T at 150+ kHz): a fixed latency of the receiver's decimation and
; filter chain, measured on a steady GSM carrier. So the sweep retunes right after every RSSI update and
; files the (L+1)-th update after a retune under that point: about 1 T per point, where waiting the
; latency out at every point took ~4 T. L (2 below 17 kHz, 3 below 150 kHz, 4 above) is what matched a slow
; per-point reference best, as accurate as it or better (docs/measurements.md of sxplug). Point 0 is
; retuned right after an update too, like the others, and then gets 2 more updates: the radio comes from the
; previous sweep's last point, a whole span away, and a jump of several MHz settles 1-2 updates slower than a
; step. That costs 3 updates per sweep.
;
;   host: put the chip in GFSK RX (RX bandwidth = RBW), then
;         0x9B  F3 F2 F1 F0  T2 T1 T0  P  L  D  W
;           F = start frequency, BE32, in REG_FREQ units (MHz * 2^20)
;           T = step, BE24, same units
;           P = points 1..255, L = updates skipped after a retune: a point gets the (L+1)-th (P + L <= 255)
;           D = delay loops between RSSI reads,  W = reads before giving up waiting for an update (1..255)
;   result: RADIO_BUF[i] = RSSI of point i, -2 * dBm like GetRssiInst. Holds BUSY for the whole sweep.

SX_TAG  = 'P'

; parameters (short addresses)
P_FREQ  = 0x9A                  ; 4 bytes
P_STEP  = 0x9E                  ; 3 bytes
P_NUM   = 0xA1
P_LAT   = 0xA2
P_DELAY = 0xA3
P_WAIT  = 0xA4
; scratch, past the parameters (PARAMS is ours until the next command)
CUR     = 0xA5                  ; 4 bytes, frequency of the point being tuned
UPD     = 0xA9                  ; RSSI updates left to read (P + L)
TUNE    = 0xAA                  ; retunes left (P - 1)
SKIP    = 0xAB                  ; updates left before results start (L)
LAST    = 0xAC                  ; last RSSI value seen

on_9b:
        spush   ip
        spush   i2
        move    r3, [P_NUM]
        jzs     .bad
        move    r2, r3
        dec     r2, r2
        move    [TUNE], r2
        add     r3, [P_LAT]
        jcs     .bad                    ; P + L > 255
        move    [UPD], r3
        move    r3, [P_LAT]
        move    [SKIP], r3
        move    r3, [P_WAIT]
        jzs     .bad
        move    r3, [P_FREQ]
        move    [CUR], r3
        move    r3, [P_FREQ + 1]
        move    [CUR + 1], r3
        move    r3, [P_FREQ + 2]
        move    [CUR + 2], r3
        move    r3, [P_FREQ + 3]
        move    [CUR + 3], r3
        ld16    i0, REG_RSSI_INST
        ld16    i1, RADIO_BUF
        move    r3, (i0)
        move    [LAST], r3
        calls   wait_update             ; sync to the update grid
        calls   retune                  ; point 0
        calls   wait_update             ; settle the jump back from the last point
        calls   wait_update

.update:
        calls   wait_update             ; -> r3
        move    r2, [SKIP]
        jzs     .store
        dec     r2, r2                  ; still inside the latency: belongs to no point
        move    [SKIP], r2
        jump    .tune
.store: move    (i1)+, r3
.tune:  move    r3, [TUNE]
        jzs     .count
        dec     r3, r3
        move    [TUNE], r3
        move    r3, [CUR + 3]           ; next point: CUR += STEP
        add     r3, [P_STEP + 2]
        move    [CUR + 3], r3
        move    r3, [CUR + 2]
        addc    r3, [P_STEP + 1]
        move    [CUR + 2], r3
        move    r3, [CUR + 1]
        addc    r3, [P_STEP]
        move    [CUR + 1], r3
        move    r3, [CUR]
        addc    r3, #0
        move    [CUR], r3
        calls   retune
.count: move    r3, [UPD]
        dec     r3, r3
        move    [UPD], r3
        jzc     .update

        move    r2, #ST_OK
        jump    .out
.bad:
        move    r2, #ST_ERROR
.out:
        move    r3, #0
        spop    i2
        spop    ip
        rets

retune:                                 ; CUR -> REG_FREQ, MSB first
        move    r3, [CUR]
        move    [0x6B], r3
        move    r3, [CUR + 1]
        move    [0x6C], r3
        move    r3, [CUR + 2]
        move    [0x6D], r3
        move    r3, [CUR + 3]
        move    [0x6E], r3              ; commits the new frequency
        rets

; waits for the RSSI register to change (= a new update), at most P_WAIT reads.
; -> r3 = the new value. Uses r2, i2l.
wait_update:
        move    r2, [P_WAIT]
.read:
        move    r3, (i0)
        cmp     r3, [LAST]
        jne     .new
        move    i2l, [P_DELAY]
        jzs     .next
.delay:
        dec     i2l, i2l
        jzc     .delay
.next:
        dec     r2, r2
        jzc     .read
.new:
        move    [LAST], r3
        rets
