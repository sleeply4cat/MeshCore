"""CoolRISC 816 instruction set: encoding tables, decoder and canonical text form.

Source of truth: CoolRISC 816 Reference Manual v4.5 (Xemics), tables 2.4/2.5.
The decoder is strict: anything that does not match an encoding exactly is a
`.word`, so `asm.encode_line(disasm(w)) == w` holds for every 22-bit word
(checked by `python -m sxplug selftest`).

Syntax is the manual's, with `[addr]` for 8-bit direct addressing.

One deliberate difference from the earlier research listings: the register
form of `cmp`/`cmpa` prints the operand from bits 11:8 first. With that order
`cmp d, s` behaves exactly as the manual describes for every form
(C = 1 iff d <= s, a = s - d) -- this is what the hardware does (checked by
random-instruction runs against the chip) and what the Semtech patch relies on
(its delay loop `cmp r3, iph; ... jvs` loops while r3:r2 < ip).
"""

REGN = ['i0l', 'i0h', 'i1l', 'i1h', 'i2l', 'i2h', 'i3l', 'i3h',
        'ipl', 'iph', 'stat', 'r3', 'r2', 'r1', 'r0', 'a']
REG = {n: i for i, n in enumerate(REGN)}
R_A, R_STAT, R_IPL, R_IPH, R_R3, R_R2 = 15, 10, 8, 9, 11, 12

ALU5 = {0: 'cmpa', 1: 'cmp', 2: 'and', 3: 'subs', 4: 'subd', 5: 'subdc', 6: 'mula',
        7: 'subsc', 8: 'xor', 10: 'move', 11: 'or', 12: 'add', 13: 'addc', 14: 'mul',
        16: 'shra', 17: 'inc', 18: 'cmvd', 19: 'cmvs', 20: 'shrc', 21: 'incc',
        22: 'shr', 24: 'cpl1', 25: 'cpl2', 26: 'shl', 27: 'dec', 28: 'cpl2c',
        30: 'shlc', 31: 'decc'}
ALU4 = {0: 'cmpa', 1: 'cmp', 2: 'and', 3: 'subs', 4: 'subd', 5: 'subdc', 6: 'mula',
        7: 'subsc', 8: 'xor', 10: 'move', 11: 'or', 12: 'add', 13: 'addc', 14: 'mul',
        15: 'tstb'}
ALU5_BY = {v: k for k, v in ALU5.items()}
ALU4_BY = {v: k for k, v in ALU4.items()}

# register-form shapes (manual: an unused register field is 1111)
UNARY = {'move', 'cmvd', 'cmvs', 'cpl1', 'cpl2', 'cpl2c', 'inc', 'incc', 'dec',
         'decc', 'shl', 'shlc', 'shr', 'shrc', 'shra'}   # op ri, rj      (k = 1111)
NORES = {'cmp', 'cmpa'}                                 # op rk, rj      (i = 1111)

CC = ['jcc', 'jvc', 'jzc', 'jump', 'jcs', 'jvs', 'jzs', 'jev']
CC_BY = {n: i for i, n in enumerate(CC)}
# manual aliases, valid after `cmp d, s`: jgt d>s, jge d>=s, jlt d<s, jle d<=s
CC_ALIAS = {'jgt': 0, 'jge': 1, 'jne': 2, 'jle': 4, 'jlt': 5, 'jeq': 6}

FIXED = {0x3FFFFF: 'nop', 0x3F3FFF: 'ret', 0x3F1FFF: 'reti', 0x3EBFFF: 'pop',
         0x2DFFFF: 'push', 0x2AFFFF: 'calls ip', 0x29FFFF: 'call ip',
         0x0BDFFF: 'halt', 0x0B7FFF: 'sflag', 0x0BEEFF: 'pmd #0', 0x0BEFFF: 'pmd #1'}
FIXED_BY = {v: k for k, v in FIXED.items()}

NOP, JUMP_IP = 0x3FFFFF, 0x23FFFF
MASK = 0x3FFFFF


def decode(w):
    """-> dict describing the instruction, or None for an invalid encoding.

    kinds: fixed names ('nop', 'ret', 'calls ip', ...), 'call'/'calls' (t),
    'j' (cc, t), 'jip' (cc), 'alu' (op, reg, src=('imm', v)|('mem', ea)),
    'alur' (op, i, j, kk), 'freq' (n), 'st' (reg, ea), 'stimm' (addr, v).
    ea = (kind, ix, n): ('off', ix, offset) ('post', ix, n) ('pre', ix, n)
         ('r3', ix, 0) ('dir', None, addr)
    """
    w &= MASK
    lo16, lo8 = w & 0xffff, w & 0xff
    if w in FIXED:
        return {'k': FIXED[w]}
    top3, top5, top6 = w >> 19, w >> 17, w >> 16
    if top6 in (0x39, 0x3A):
        return {'k': 'call' if top6 == 0x39 else 'calls', 't': ~lo16 & 0xffff}
    if top3 == 6:
        return {'k': 'j', 'cc': top6 & 7, 't': ~lo16 & 0xffff}
    if top3 == 4 and lo16 == 0xffff:
        return {'k': 'jip', 'cc': top6 & 7}
    reg, op5 = (w >> 8) & 0xf, (w >> 12) & 0x1f
    if top3 in (2, 3):
        op = ALU5.get(op5)
        if not op:
            return None
        ix = (w >> 17) & 3
        if top3 == 3:
            ea = ('off', ix, lo8)
        elif lo8 & 0x80:
            ea = ('pre', ix, 0x80 - (lo8 & 0x7f))
        else:
            ea = ('post', ix, lo8 & 0x7f)
        return {'k': 'alu', 'op': op, 'reg': reg, 'src': ('mem', ea)}
    if top6 == 0x0E:
        op = ALU4.get((w >> 12) & 0xf)
        if op is None:
            return None                       # alu_op:4 = 9 is undefined
        if op == 'tstb':
            m = ~lo8 & 0xff                   # n_data = ~(1 << bit)  (hw-checked)
            if not (m and not m & (m - 1)):
                return None
            return {'k': 'alu', 'op': 'tstb', 'reg': reg, 'src': ('imm', m)}
        return {'k': 'alu', 'op': op, 'reg': reg, 'src': ('imm', ~lo8 & 0xff)}
    if top5 == 6:
        op = ALU5.get(op5)
        if not op:
            return None
        return {'k': 'alur', 'op': op, 'i': w & 0xf, 'j': (w >> 4) & 0xf, 'kk': reg}
    if top6 == 0x0B and (w & 0xfff0) == 0xBFF0:
        return {'k': 'freq', 'n': w & 0xf}
    if top5 == 3 and (lo8 & 0xfc) == 0xfc:
        op = ALU5.get(op5)
        return {'k': 'alu', 'op': op, 'reg': reg, 'src': ('mem', ('r3', w & 3, 0))} if op else None
    if top5 == 2:
        op = ALU5.get(op5)
        return {'k': 'alu', 'op': op, 'reg': reg, 'src': ('mem', ('dir', None, ~lo8 & 0xff))} if op else None
    top8 = w >> 14
    ix = (w >> 12) & 3
    if top8 == 0x0E and lo8 == 0xff:
        return {'k': 'st', 'reg': reg, 'ea': ('r3', ix, 0)}
    if top8 == 0x0D:
        ea = ('pre', ix, 0x80 - (lo8 & 0x7f)) if lo8 & 0x80 else ('post', ix, lo8 & 0x7f)
        return {'k': 'st', 'reg': reg, 'ea': ea}
    if top8 == 0x0B:
        return {'k': 'st', 'reg': reg, 'ea': ('off', ix, lo8)}
    if w >> 12 == 0x1B:
        return {'k': 'st', 'reg': reg, 'ea': ('dir', None, ~lo8 & 0xff)}
    if top6 == 0:
        return {'k': 'stimm', 'addr': ~lo8 & 0xff, 'v': ~(w >> 8) & 0xff}
    return None


def _ea(ea):
    kind, ix, n = ea
    i = 'i%d' % ix if ix is not None else None
    if kind == 'dir':
        return '[0x%02x]' % n
    if kind == 'r3':
        return '(%s,r3)' % i
    if kind == 'off':
        return '(%s)' % i if n == 0 else '(%s,0x%02x)' % (i, n)
    if kind == 'post':
        return '(%s)+' % i if n == 1 else '(%s,0x%02x)+' % (i, n)
    return '-(%s)' % i if n == 1 else '-(%s,0x%02x)' % (i, n)


def disasm(w):
    """-> (text, ref); ref = ('call'|'jmp', target, cc) for direct transfers, else None."""
    w &= MASK
    d = decode(w)
    if d is None:
        return '.word 0x%06x' % w, None
    k = d['k']
    r = REGN
    if k in FIXED_BY:
        return k, None
    if k in ('call', 'calls'):
        return '%s 0x%04x' % (k, d['t']), ('call', d['t'], None)
    if k == 'j':
        return '%s 0x%04x' % (CC[d['cc']], d['t']), ('jmp', d['t'], d['cc'])
    if k == 'jip':
        return '%s ip' % CC[d['cc']], None
    if k == 'freq':
        return 'freq #%d' % d['n'], None
    if k == 'alu':
        op, reg, (sk, sv) = d['op'], d['reg'], d['src']
        if op == 'tstb':
            return 'tstb %s, #%d' % (r[reg], sv.bit_length() - 1), None
        if sk == 'imm':
            return '%s %s, #0x%02x' % (op, r[reg], sv), None
        return '%s %s, %s' % (op, r[reg], _ea(sv)), None
    if k == 'alur':
        op, i, j, kk = d['op'], d['i'], d['j'], d['kk']
        if op in UNARY and kk == 0xf:
            return '%s %s, %s' % (op, r[i], r[j]), None
        if op in NORES and i == 0xf:
            return '%s %s, %s' % (op, r[kk], r[j]), None      # d (bits 11:8), s (bits 7:4)
        if op not in UNARY | NORES and kk == i:
            return '%s %s, %s' % (op, r[i], r[j]), None
        return '%s %s, %s, %s' % (op, r[i], r[j], r[kk]), None
    if k == 'st':
        return 'move %s, %s' % (_ea(d['ea']), r[d['reg']]), None
    if k == 'stimm':
        return 'move [0x%02x], #0x%02x' % (d['addr'], d['v']), None
    raise AssertionError(k)
