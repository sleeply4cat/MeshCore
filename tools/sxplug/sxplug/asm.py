"""CoolRISC 816 assembler.

Strict inverse of `isa.disasm` (every disassembled word re-assembles to itself),
plus what is needed to write programs by hand:

    label:                       code address (word address, PRAM base 0x8000)
    .local:                      label local to the previous plain label (`jump .local`)
    NAME = expr | .equ NAME, expr      constant
    .default NAME, expr          define NAME only if the program does not define it
    .include "file"              textual include (relative to the including file,
                                 then to the include path; the SDK's include/ is
                                 always on it)
    .org expr                    move forward, padding with nop
    .word expr[, expr...]        raw 22-bit words
    .assert expr[, "message"]    fail the build when expr == 0

Expressions: Python-like integer arithmetic on numbers (123, 0x7B, 0b101, 'R'),
symbols, + - * / % << >> & | ^ ~, comparisons (== != < <= > >=, give 0/1), parentheses, lo(x) = x & 0xff, hi(x) = (x >> 8) & 0xff.

Instructions use the manual's syntax (see docs/isa.md); mnemonics are case-insensitive,
register names are lower case:

    move r2, #0x08        move r3, (i0,0x02)     move (i0)+, a     move [0x97], #2
    and r3, #~0x08 & 0xff cmp r3, #0x20          jle done          calls sub   jump ip

Pseudo-instructions (expand to real ones):

    rets                  = jump ip           (return from a `calls`)
    jeq jne jgt jge jlt jle = jzs jzc jcc jvc jvs jcs  (meaning after `cmp d, s`)
    ld16 iN|ip, expr      = move iNl, #lo(expr); move iNh, #hi(expr)
    spush X / spop X      = move -(i3), X / move X, (i3)+  (software stack on i3);
                            for X = i0/i1/i2/ip both halves, low byte pushed first,
                            so `spush ip ... spop ip` pairs up
"""
import ast, os, re

from . import isa
from .isa import REG, REGN, ALU5_BY, ALU4_BY, CC_BY, CC_ALIAS, FIXED_BY, UNARY, NORES

SDK_INCLUDE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'include')


class AsmError(Exception):
    def __init__(self, msg, where=None):
        self.where = where
        super().__init__('%s: %s' % (where, msg) if where else msg)


# ---------------------------------------------------------------- expressions
_CHAR = re.compile(r"'(\\.|[^\\'])'")
_ESC = {'\\n': 10, '\\r': 13, '\\t': 9, '\\0': 0, "\\'": 39, '\\\\': 92}
_BINOPS = {ast.Add: lambda a, b: a + b, ast.Sub: lambda a, b: a - b,
           ast.Mult: lambda a, b: a * b, ast.Div: lambda a, b: a // b,
           ast.FloorDiv: lambda a, b: a // b, ast.Mod: lambda a, b: a % b,
           ast.LShift: lambda a, b: a << b, ast.RShift: lambda a, b: a >> b,
           ast.BitAnd: lambda a, b: a & b, ast.BitOr: lambda a, b: a | b,
           ast.BitXor: lambda a, b: a ^ b}
_CMPOPS = {ast.Eq: lambda a, b: a == b, ast.NotEq: lambda a, b: a != b,
           ast.Lt: lambda a, b: a < b, ast.LtE: lambda a, b: a <= b,
           ast.Gt: lambda a, b: a > b, ast.GtE: lambda a, b: a >= b}
_FUNCS = {'lo': lambda x: x & 0xff, 'hi': lambda x: (x >> 8) & 0xff}


def _chars(s):
    def rep(m):
        t = m.group(1)
        return str(_ESC[t] if t in _ESC else ord(t))
    return _CHAR.sub(rep, s)


def eval_expr(text, lookup):
    """Evaluate an integer expression; lookup(name) -> int (or raises AsmError)."""
    src = _chars(text.strip())
    if not src:
        raise AsmError('empty expression')
    try:
        tree = ast.parse(src, mode='eval').body
    except SyntaxError:
        raise AsmError('bad expression %r' % text)

    def ev(n):
        if isinstance(n, ast.Constant) and type(n.value) is int:
            return n.value
        if isinstance(n, ast.Name):
            return lookup(n.id)
        if isinstance(n, ast.UnaryOp):
            v = ev(n.operand)
            if isinstance(n.op, ast.USub): return -v
            if isinstance(n.op, ast.UAdd): return v
            if isinstance(n.op, ast.Invert): return ~v
        if isinstance(n, ast.BinOp) and type(n.op) in _BINOPS:
            return _BINOPS[type(n.op)](ev(n.left), ev(n.right))
        if isinstance(n, ast.Compare) and len(n.ops) == 1 and type(n.ops[0]) in _CMPOPS:
            return int(_CMPOPS[type(n.ops[0])](ev(n.left), ev(n.comparators[0])))
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in _FUNCS
                and len(n.args) == 1 and not n.keywords):
            return _FUNCS[n.func.id](ev(n.args[0]))
        raise AsmError('unsupported expression %r' % text)
    return ev(tree)


# ---------------------------------------------------------------- operands
_IX = r'i([0-3])'


def parse_operand(s):
    """-> ('reg', n) | ('ip',) | ('imm', expr) | ('dir', expr) | ('off', ix, expr|None)
          | ('post', ix, expr|None) | ('pre', ix, expr|None) | ('r3', ix) | ('expr', expr)"""
    s = s.strip()
    if s in REG:
        return ('reg', REG[s])
    if s == 'ip':
        return ('ip',)
    if s.startswith('#'):
        return ('imm', s[1:])
    m = re.fullmatch(r'\[(.*)\]', s)
    if m:
        return ('dir', m.group(1))
    m = re.fullmatch(r'\(\s*' + _IX + r'\s*,\s*r3\s*\)', s)
    if m:
        return ('r3', int(m.group(1)))
    m = re.fullmatch(r'-\s*\(\s*' + _IX + r'\s*(?:,(.*))?\)', s)
    if m:
        return ('pre', int(m.group(1)), m.group(2))
    m = re.fullmatch(r'\(\s*' + _IX + r'\s*(?:,(.*))?\)\s*\+', s)
    if m:
        return ('post', int(m.group(1)), m.group(2))
    m = re.fullmatch(r'\(\s*' + _IX + r'\s*(?:,(.*))?\)', s)
    if m:
        return ('off', int(m.group(1)), m.group(2))
    return ('expr', s)


def split_operands(rest):
    out, depth, cur, q = [], 0, '', False
    for ch in rest:
        if ch == "'":
            q = not q
        if not q:
            if ch in '([':
                depth += 1
            elif ch in ')]':
                depth -= 1
            if ch == ',' and depth == 0:
                out.append(cur.strip()); cur = ''; continue
        cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


# ---------------------------------------------------------------- encoder
def _rng(v, lo, hi, what):
    if not lo <= v <= hi:
        raise AsmError('%s out of range: %d' % (what, v))
    return v


def encode(mn, ops, val):
    """Encode one real instruction. val(expr) -> int. Returns a 22-bit word."""
    n = len(ops)
    if n == 0 and mn in FIXED_BY:
        return FIXED_BY[mn]
    if mn == 'pmd' and n == 1 and ops[0][0] == 'imm':
        s = val(ops[0][1])
        if s not in (0, 1): raise AsmError('pmd #0 or #1')
        return 0x0BEFFF if s else 0x0BEEFF
    if mn == 'freq' and n == 1 and ops[0][0] == 'imm':
        return 0x0BBFF0 | _rng(val(ops[0][1]), 0, 15, 'freq divider')
    if mn in ('call', 'calls') and n == 1:
        if ops[0][0] == 'ip':
            return FIXED_BY[mn + ' ip']
        if ops[0][0] == 'expr':
            t = _rng(val(ops[0][1]), 0, 0xffff, 'address')
            return (0x39 if mn == 'call' else 0x3A) << 16 | (~t & 0xffff)
    cc = CC_BY.get(mn, CC_ALIAS.get(mn))
    if cc is not None and n == 1:
        if ops[0][0] == 'ip':
            return (0x20 + cc) << 16 | 0xFFFF
        if ops[0][0] == 'expr':
            t = _rng(val(ops[0][1]), 0, 0xffff, 'address')
            return (0x30 + cc) << 16 | (~t & 0xffff)
    if mn == 'tstb' and n == 2 and ops[0][0] == 'reg' and ops[1][0] == 'imm':
        b = _rng(val(ops[1][1]), 0, 7, 'bit number')
        return 0x0E0000 | 0xF << 12 | ops[0][1] << 8 | (~(1 << b) & 0xff)
    # register <- immediate (alu4)
    if n == 2 and ops[0][0] == 'reg' and ops[1][0] == 'imm' and mn in ALU4_BY and mn != 'tstb':
        v = _rng(val(ops[1][1]), -128, 255, 'immediate') & 0xff
        return 0x0E0000 | ALU4_BY[mn] << 12 | ops[0][1] << 8 | (~v & 0xff)
    # stores
    if mn == 'move' and n == 2 and ops[0][0] in ('off', 'post', 'pre', 'r3', 'dir'):
        dst, src = ops
        if dst[0] == 'dir' and src[0] == 'imm':
            a = _rng(val(dst[1]), 0, 255, 'direct address')
            v = _rng(val(src[1]), -128, 255, 'immediate') & 0xff
            return (~v & 0xff) << 8 | (~a & 0xff)
        if src[0] != 'reg':
            raise AsmError('store source must be a register (or #imm for [addr])')
        r = src[1]
        if dst[0] == 'off':
            o = _rng(val(dst[2]) if dst[2] else 0, 0, 255, 'offset')
            return 0x0B << 14 | dst[1] << 12 | r << 8 | o
        if dst[0] in ('post', 'pre'):
            return 0x0D << 14 | dst[1] << 12 | r << 8 | _modc(dst, val)
        if dst[0] == 'r3':
            return 0x0E << 14 | dst[1] << 12 | r << 8 | 0xFF
        a = _rng(val(dst[1]), 0, 255, 'direct address')
        return 0x1B << 12 | r << 8 | (~a & 0xff)
    # register <- memory (alu5)
    if n == 2 and ops[0][0] == 'reg' and mn in ALU5_BY and ops[1][0] in ('off', 'post', 'pre', 'r3', 'dir'):
        op5, r, src = ALU5_BY[mn], ops[0][1], ops[1]
        if src[0] == 'off':
            o = _rng(val(src[2]) if src[2] else 0, 0, 255, 'offset')
            return 0x3 << 19 | src[1] << 17 | op5 << 12 | r << 8 | o
        if src[0] in ('post', 'pre'):
            return 0x2 << 19 | src[1] << 17 | op5 << 12 | r << 8 | _modc(src, val)
        if src[0] == 'r3':
            return 0x3 << 17 | op5 << 12 | r << 8 | 0xFC | src[1]
        a = _rng(val(src[1]), 0, 255, 'direct address')
        return 0x2 << 17 | op5 << 12 | r << 8 | (~a & 0xff)
    # register form (alu5)
    if n in (2, 3) and all(o[0] == 'reg' for o in ops) and mn in ALU5_BY:
        op5 = ALU5_BY[mn]
        rr = [o[1] for o in ops]
        if n == 3:
            i, j, k = rr
        elif mn in UNARY:
            i, j, k = rr[0], rr[1], 0xF
        elif mn in NORES:
            i, j, k = 0xF, rr[1], rr[0]          # cmp d, s: d in bits 11:8
        else:
            i, j, k = rr[0], rr[1], rr[0]
        return 6 << 17 | op5 << 12 | k << 8 | j << 4 | i
    raise AsmError('cannot encode: %s %s' % (mn, ', '.join(_show(o) for o in ops)))


def _modc(ea, val):
    v = val(ea[2]) if ea[2] else 1
    if ea[0] == 'post':
        return _rng(v, 0, 0x7f, 'post-increment')
    _rng(v, 1, 0x80, 'pre-decrement')
    return 0x80 | ((0x80 - v) & 0x7f)


def _show(o):
    return {'reg': lambda: REGN[o[1]], 'ip': lambda: 'ip'}.get(o[0], lambda: str(o[1:]))()


# ---------------------------------------------------------------- pseudo-ops
_PAIRS = {'i0': ('i0l', 'i0h'), 'i1': ('i1l', 'i1h'), 'i2': ('i2l', 'i2h'),
          'i3': ('i3l', 'i3h'), 'ip': ('ipl', 'iph')}


def expand(mn, raw):
    """Pseudo-instruction -> list of (mnemonic, [operand text]) or None if not a pseudo."""
    if mn == 'rets' and not raw:
        return [('jump', ['ip'])]
    if mn == 'ld16' and len(raw) == 2 and raw[0] in _PAIRS:
        lo, hi = _PAIRS[raw[0]]
        e = raw[1]
        return [('move', [lo, '#lo(%s)' % e]), ('move', [hi, '#hi(%s)' % e])]
    if mn in ('spush', 'spop') and len(raw) == 1:
        x = raw[0]
        if x in ('i0', 'i1', 'i2', 'ip'):
            lo, hi = _PAIRS[x]
            if mn == 'spush':
                return [('move', ['-(i3)', lo]), ('move', ['-(i3)', hi])]
            return [('move', [hi, '(i3)+']), ('move', [lo, '(i3)+'])]
        if x in REG and x not in ('i3l', 'i3h'):
            return [('move', ['-(i3)', x])] if mn == 'spush' else [('move', [x, '(i3)+'])]
        raise AsmError('cannot %s %s' % (mn, x))
    return None


def encode_line(text, symbols=None):
    """Assemble a single instruction line (no labels/directives). Used by selftest."""
    symbols = symbols or {}

    def val(e):
        return eval_expr(e, lambda nm: symbols[nm] if nm in symbols else _undef(nm))
    mn, raw = _split(text)
    if mn == '.word' and len(raw) == 1:
        return val(raw[0]) & isa.MASK
    ex = expand(mn, raw) or [(mn, raw)]
    out = [encode(m, [parse_operand(o) for o in r], val) for m, r in ex]
    return out[0] if len(out) == 1 else out


def _undef(nm):
    raise AsmError('undefined symbol %r' % nm)


def _split(text):
    m = re.match(r'^([A-Za-z_.][\w.]*)\s*(.*)$', text.strip())
    if not m:
        raise AsmError('bad line %r' % text)
    return m.group(1).lower(), split_operands(m.group(2))


# ---------------------------------------------------------------- program
_LOCALREF = re.compile(r"(?<![\w.'])\.[A-Za-z_]\w*")


def _local(scope, name):
    return '%s__%s' % (scope, name[1:])


class Program:
    def __init__(self, base):
        self.base = base
        self.words = []           # contiguous from base
        self.symbols = {}         # name -> int (labels and constants)
        self.labels = set()
        self.lines = []           # (addr, [words], file, line, text)

    @property
    def end(self):
        return self.base + len(self.words)


def _strip_comment(line):
    out, q = '', False
    for ch in line:
        if ch == "'":
            q = not q
        if ch == ';' and not q:
            break
        out += ch
    return out.rstrip()


def _read_source(path, text, include_dirs, seen):
    """Yield (file, lineno, text) with includes expanded."""
    if text is None:
        with open(path) as f:
            text = f.read()
    for no, line in enumerate(text.splitlines(), 1):
        code = _strip_comment(line).strip()
        m = re.match(r'^\.include\s+"([^"]+)"$', code)
        if not m:
            yield path, no, code
            continue
        name = m.group(1)
        cands = [] if path.startswith('<') else [os.path.join(os.path.dirname(os.path.abspath(path)), name)]
        cands += [os.path.join(d, name) for d in include_dirs]
        found = next((c for c in cands if os.path.isfile(c)), None)
        if not found:
            raise AsmError('include not found: %s' % name, '%s:%d' % (path, no))
        found = os.path.abspath(found)
        if found in seen:
            raise AsmError('recursive include: %s' % name, '%s:%d' % (path, no))
        yield from _read_source(found, None, include_dirs, seen | {found})


def assemble(text=None, path='<src>', base=0x8000, include_dirs=(), defines=None):
    """Assemble source text (or the file at `path`). Returns a Program."""
    incs = list(include_dirs) + [SDK_INCLUDE]
    stmts = list(_read_source(path, text, incs, {os.path.abspath(path)} if text is None else set()))
    prog = Program(base)
    exprs = dict((k, str(v)) for k, v in (defines or {}).items())   # name -> expr (constants)
    where_of = {k: 'command line' for k in exprs}
    defaults = []
    asserts = []
    items = []                 # (addr, kind, payload, where, text)
    addr = base

    def resolver(stack=()):
        def look(nm):
            if nm in prog.symbols:
                return prog.symbols[nm]
            if nm in exprs:
                if nm in stack:
                    raise AsmError('circular definition of %r' % nm)
                v = eval_expr(exprs[nm], resolver(stack + (nm,)))
                prog.symbols[nm] = v
                return v
            raise AsmError('undefined symbol %r' % nm)
        return look

    def define(nm, where):
        if nm in where_of:
            raise AsmError('symbol %r already defined (%s)' % (nm, where_of.get(nm, 'earlier')), where)
        if nm in REG or nm in ('ip', 'i0', 'i1', 'i2', 'i3', 'lo', 'hi'):
            raise AsmError('reserved name %r' % nm, where)
        where_of[nm] = where

    # pass 1: addresses, labels, constants
    scope = ''
    for f, no, code in stmts:
        where = '%s:%d' % (os.path.basename(f), no)
        if not code:
            continue
        orig = code
        try:
            m = re.match(r'^(\.?[A-Za-z_][\w]*):\s*(.*)$', code)
            if m:
                name = m.group(1)
                if name.startswith('.'):
                    if not scope:
                        raise AsmError('local label %s before any label' % name)
                    name = _local(scope, name)
                else:
                    scope = name
                define(name, where)
                prog.symbols[name] = addr
                prog.labels.add(name)
                code = orig = m.group(2).strip()
                if not code:
                    continue
            if scope and '.' in code[1:]:
                code = code[:1] + _LOCALREF.sub(lambda mm: _local(scope, mm.group(0)), code[1:])
            m = re.match(r'^([A-Za-z_]\w*)\s*=\s*(.+)$', code) or \
                re.match(r'^\.equ\s+([A-Za-z_]\w*)\s*,\s*(.+)$', code)
            if m:
                define(m.group(1), where)
                exprs[m.group(1)] = m.group(2)
                continue
            m = re.match(r'^\.default\s+([A-Za-z_]\w*)\s*,\s*(.+)$', code)
            if m:
                defaults.append((m.group(1), m.group(2), where))
                continue
            m = re.match(r'^\.assert\s+(.+?)(?:\s*,\s*"([^"]*)")?$', code)
            if m:
                asserts.append((m.group(1), m.group(2), where))
                continue
            m = re.match(r'^\.org\s+(.+)$', code)
            if m:
                t = eval_expr(m.group(1), resolver())
                if t < addr:
                    raise AsmError('.org 0x%04x goes backwards (at 0x%04x)' % (t, addr))
                items.append((addr, 'org', t, where, code))
                addr = t
                continue
            m = re.match(r'^\.word\s+(.+)$', code)
            if m:
                ws = split_operands(m.group(1))
                items.append((addr, 'word', ws, where, code))
                addr += len(ws)
                continue
            if code.startswith('.'):
                raise AsmError('unknown directive %r' % code.split()[0])
            mn, raw = _split(code)
            ex = expand(mn, raw) or [(mn, raw)]
            items.append((addr, 'insn', ex, where, orig))
            addr += len(ex)
        except AsmError as e:
            raise AsmError(str(e), where) if e.where is None else e
    for nm, e, where in defaults:
        if nm not in where_of and nm not in exprs:
            exprs[nm] = e
            where_of[nm] = where
    look = resolver()

    # pass 2: encode
    for a, kind, payload, where, text in items:
        try:
            if kind == 'org':
                prog.words += [isa.NOP] * (payload - (base + len(prog.words)))
                continue
            if kind == 'word':
                ws = [eval_expr(x, look) for x in payload]
                for w in ws:
                    if not 0 <= w <= isa.MASK:
                        raise AsmError('.word out of 22 bits: 0x%x' % w)
            else:
                ws = [encode(m, [parse_operand(o) for o in r], lambda e: eval_expr(e, look))
                      for m, r in payload]
            assert base + len(prog.words) == a
            prog.words += ws
            prog.lines.append((a, ws, where, text))
        except AsmError as e:
            raise AsmError(str(e), where) if e.where is None else e
    for e, msg, where in asserts:
        try:
            ok = eval_expr(e, look)
        except AsmError as err:
            raise AsmError(str(err), where)
        if not ok:
            raise AsmError('assertion failed: %s' % (msg or e), where)
    for nm in list(exprs):
        try:
            look(nm)
        except AsmError as e:
            raise AsmError(str(e), where_of.get(nm))
    return prog


def selftest(n=1 << 22, step=1):
    """asm(dis(w)) == w for every word (or every `step`-th). Returns list of failures."""
    bad = []
    for w in range(0, n, step):
        txt = isa.disasm(w)[0]
        try:
            w2 = encode_line(txt)
        except AsmError as e:
            w2 = str(e)
        if w2 != w:
            bad.append((w, txt, w2))
            if len(bad) > 20:
                break
    return bad
