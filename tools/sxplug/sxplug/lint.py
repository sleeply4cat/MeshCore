"""Static checks of a PRAM image (no execution). `sim` catches the dynamic cases."""
from . import chip, isa


def lint(img):
    """-> list of (level, addr, text); level 'error' | 'warning'."""
    out = []
    end = img.base + len(img.words)
    if len(img.words) > chip.PRAM_WORDS:
        out.append(('error', None, 'image is %d words, PRAM holds %d' % (len(img.words), chip.PRAM_WORDS)))
    if end > chip.PRAM_EXEC_CHECKED:
        out.append(('warning', None, 'code past 0x%04X: PRAM holds it, but execution there was '
                    'not tested on hw' % chip.PRAM_EXEC_CHECKED))
    for i, w in enumerate(img.words):
        a = img.base + i
        d = isa.decode(w)
        if d is None:
            if 0x801E <= a:     # data words inside code are allowed but suspicious
                out.append(('warning', a, 'not an instruction: .word 0x%06x' % w))
            continue
        k = d['k']
        if k in ('call', 'calls', 'j'):
            t = d['t']
            if not img.base <= t < end:
                where = 'ROM' if t < 0x2000 else 'nowhere'
                out.append(('warning', a, '%s to 0x%04X (%s): ROM functions are mostly undocumented'
                            % (isa.disasm(w)[0].split()[0], t, where)))
        elif k == 'stimm':
            c = chip.check_write(d['addr'], d['v'])
            if c:
                out.append((c[0], a, c[1]))
        elif k == 'st' and d['ea'][0] == 'dir':
            c = chip.check_write(d['ea'][2], None)
            if c and c[0] == 'error':
                out.append(('warning', a, c[1].replace('write unknown value to', 'register stored to')
                            + ' -- check the value'))
            elif c:
                out.append(c[:1] + (a,) + c[1:])
        elif k in ('halt', 'pmd #0', 'pmd #1', 'freq', 'reti'):
            out.append(('warning', a, '`%s`: core control, untested from a plugin' % isa.disasm(w)[0]))
        if k in ('call', 'call ip', 'push', 'pop') or (k == 'ret' and a != 0x8005):
            out.append(('warning', a, 'hardware call stack (%s): handlers use calls/jump ip' % k))
        if k in ('alu', 'alur') and d['op'] not in isa.NORES | {'tstb'} and \
                (d.get('reg') if k == 'alu' else d['i']) == isa.R_STAT:
            out.append(('warning', a, 'writes stat (interrupt/event bits)'))
        dst = d.get('reg') if k == 'alu' else d.get('i')
        if k in ('alu', 'alur') and d['op'] not in isa.NORES | {'tstb'} and dst in (6, 7):
            out.append(('warning', a, 'writes i3 (the stack pointer) directly'))
    return out
