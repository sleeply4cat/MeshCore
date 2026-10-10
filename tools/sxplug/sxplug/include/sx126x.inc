; sx126x.inc -- symbols for SX126x PRAM plugins (ROM "SX1261 V2D 2D02").
; [hw] = measured on an SX1262, [semtech] = used by Semtech's own patch,
; [guess] = inferred, not confirmed. Details and evidence: docs/hardware.md.

; ---- program memory ------------------------------------------------------
PRAM_BASE       = 0x8000        ; [hw] 2048 words, 0x8000-0x87FF
PRAM_WORDS      = 2048

; ---- handler ABI ---------------------------------------------------------
PARAMS          = 0x009A        ; [hw] parameter bytes of the command (opcode's args)
PARAMS_MAX      = 0x26          ; [guess] 0x00C0+ are ROM variables: keep params <= 38 bytes
; status returned in r2 (r3 = 0); GetStatus command-status field = r2 >> 1
ST_OK           = 0x02          ; [hw] GetStatus 0x22 in STBY_RC
ST_DATA         = 0x04          ; data available           [guess, by the encoding]
ST_TIMEOUT      = 0x06          ; command timeout          [guess]
ST_ERROR        = 0x08          ; [hw] command processing error (GetStatus 0x28)
ST_FAIL         = 0x0A          ; failure to execute       [guess]

; ---- RAM -----------------------------------------------------------------
PLUGIN_RAM      = 0x0330        ; [hw] 0x0330-0x03FF free, except 0x0380-0x0383: Calibrate(0x7F) writes there
PLUGIN_RAM_END  = 0x0400
RADIO_BUF       = 0x0400        ; [hw] 256-byte radio buffer (ReadBuffer/WriteBuffer offset 0)
VERSION_STRING  = 0x0320        ; [hw] "SX1261 V2D 2D02", plain RAM
VERSION_TAG     = 0x032E        ; [hw] last char; Semtech init writes '6' -> "2D06"

; ---- registers (16-bit address: reach them through an index register) ----
REG_RSSI_INST   = 0x08BE        ; [hw] instantaneous RSSI in GFSK RX, value = -2 * dBm, = GetRssiInst;
                                ;   updates every ~1.6/RBW ms (RBW in kHz) with the window at reset;
                                ;   a retune shows up ~3.4 updates later (receiver pipeline)
REG_RSSI_LORA   = 0x08BD        ; [hw] the same in LoRa RX (0x08BE reads 0xFF there), once per symbol
REG_SCAN_RUN    = 0x089B        ; [semtech] LBT/scan keep-running flag, reset value 1;
                                ;   the host writes 0 to stop a running loop.
                                ;   [hw] bits 4:2 are also the RSSI averaging window (0 = fastest updates)
REG_SCAN_STATUS = 0x07CD        ; [semtech] spectral-scan status byte
REG_RX_LEN      = 0x02A8        ; [hw] last packet length (GetRxBufferStatus)
REG_RX_PTR      = 0x02A9        ; [hw] last packet start offset in RADIO_BUF
REG_RX_COUNT    = 0x00DA        ; [hw] +1 per received packet (direct address)
REG_FREQ        = 0x088B        ; [hw] 4 bytes BE, freq * 2^25 / 32 MHz (SFR 0x6B-0x6E);
                                ;   [hw] writing it from a plugin retunes the radio in RX; the write of
                                ;   the LSB (0x088E) commits it, so write MSB first
REG_PKT_LORA    = 0x0706        ; [hw] 0xFF LoRa / 0x00 GFSK
REG_RX_ACTIVE   = 0x0700        ; [hw] 1 in RX, 0 in standby

; ---- more registers, found by differential snapshots (docs/registers.md) [diff] ----
REG_IRQ_MASK    = 0x058C        ; BE16, SetDioIrqParams IRQ mask
REG_DIO1_MASK   = 0x058E        ; BE16
REG_DIO2_MASK   = 0x0590        ; BE16
REG_LORA_LEN    = 0x0702        ; LoRa payload length
REG_LORA_BWSF   = 0x0703        ; [hw] (BW code << 4) | SF
REG_LORA_CFG    = 0x0704        ; b7 implicit header, b6 IQ invert, b3 LDRO, b2:0 CR
REG_LORA_PREAMB = 0x073A        ; BE16 preamble length
REG_LORA_SYNC   = 0x0740        ; 2 bytes, sync word (datasheet)
REG_GFSK_BR     = 0x06A1        ; BE24 bit rate (as the SetModulationParams argument)
REG_GFSK_FDEV   = 0x06A5        ; BE16 deviation
REG_GFSK_PREAMB = 0x06B5        ; BE16 preamble length
REG_RX_BW       = 0x0807        ; GFSK: RxBw code of SetModulationParams
REG_BUF_TX_BASE = 0x0800        ; SetBufferBaseAddress
REG_BUF_RX_BASE = 0x0801
REG_TX_POWER    = 0x0883        ; power and ramp (SetTxParams)
REG_RX_GAIN     = 0x08AC        ; 0x94 normal, 0x96 boosted (datasheet)
REG_RX_TIMEOUT  = 0x0903        ; BE24, SetRx timeout
REG_RANDOM      = 0x0819        ; 4 bytes, changes by itself (RNG, datasheet)

; ---- short-address page [0x00-0x7F] --------------------------------------
; [hw] [0xXY] for X = 1..7 is the same storage as a register:
;   X=1: Y<8 -> 0x0580+Y, Y>=8 -> 0x0940+Y-8;  X=2 -> 0x05A0+Y;  X=3 -> 0x0680+Y;
;   X=4 -> 0x0700+Y;  X=5 -> 0x0800+Y;  X=6 -> 0x0880+Y;  X=7 -> 0x0900+Y
SFR_DIO_OUT     = 0x11          ; =0x0581 [semtech] bit n = level of DIO n (n = 1..3)
                                ;   [hw] bit 0 is the BUSY line: clear it to let a handler run in the
                                ;   background (BUSY drops within ~5us); setting it from the host hangs BUSY
SFR_DIO_CFG     = 0x17          ; =0x0587 [semtech] bit 4+n = output enable of DIO n
                                ;   ([hw] SetDIO2AsRfSwitchCtrl sets bit 6)
SFR_CMD_FLAGS   = 0x1C          ; =0x0944 bit 0 set by the ROM per command; [hw] it does NOT control BUSY
SFR_STATUS      = 0x21          ; =0x05A1 [hw] chip status/mode. DO NOT WRITE (0x5x = TX)
SFR_OPCODE      = 0x22          ; =0x05A2 [hw] read-only
CMD_RESULT      = 0x97          ; [hw] ROM stores the handler's r2 here
CMD_INDEX       = 0xF8          ; [hw] ROM stores opcode & 0x3F here before dispatch

; ---- ROM variables that must keep their reset values [hw] ---------------
ROM_VAR_02AE    = 0x02AE        ; reset 00 -- any other value hangs the next GetStatus
ROM_VAR_02AF    = 0x02AF        ; reset 01 -- same
STACK_FLOOR     = 0x02B0        ; lowest usable stack byte (0x02A8-0x02AF are ROM variables)

; ---- never write (wedges the chip until NRESET) [hw] ---------------------
; 0x05D4 any; 0x0900/[0x70] keep bits 4-5 set; 0x0921 keep bit 2 set;
; 0x090D/[0x7D] bit 3; 0x05A0/[0x20] bit 1; 0x0951 bit 0; 0x0581/[0x11] bit 0;
; 0x0587/[0x17] bit 2 unless bit 0, 1 or 3 is set; 0x091C bit 0; 0x091D bit 6; 0x0919.
; `sxplug sim` and `sxplug lint` flag these.
