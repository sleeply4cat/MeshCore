# PlatformIO pre-script: rebuilds the SX126x PRAM plugins with the vendored sxplug assembler
# (tools/sxplug: assembler + linter only), src/helpers/radiolib/sx126x_plugins/X.asm -> X.h, whenever
# the header is missing or the .asm (or the assembler) is newer. Assembler or lint errors stop the build.
# The headers are build output (.gitignore'd).

import glob
import os
import subprocess
import sys

Import("env")

project = env.subst("$PROJECT_DIR")
tools = os.path.join(project, "tools", "sxplug")
plugins = os.path.join(project, "src", "helpers", "radiolib", "sx126x_plugins")


def newest_mtime(top):
    newest = 0
    for dirpath, _, files in os.walk(top):
        for f in files:
            if not f.endswith(".pyc"):
                newest = max(newest, os.path.getmtime(os.path.join(dirpath, f)))
    return newest


assembler_mtime = newest_mtime(os.path.join(tools, "sxplug"))

# run in a separate interpreter, so the assembler's modules never mix with PlatformIO's
BUILD = """
import sys
from sxplug import image, lint
src, stem = sys.argv[1], sys.argv[2]
try:
    img = image.build(src)
except Exception as e:      # assembler errors carry file:line
    print('error: %s' % e)
    sys.exit(1)
found = lint.lint(img)
for level, addr, text in found:
    print('%s: 0x%04X: %s' % (level, addr, text) if addr is not None else '%s: %s' % (level, text))
if any(f[0] == 'error' for f in found):
    sys.exit(1)
img.save(stem, ['h'])
print('%d words' % len(img))
"""

for asm in sorted(glob.glob(os.path.join(plugins, "*.asm"))):
    stem = os.path.splitext(asm)[0]
    header = stem + ".h"
    if os.path.exists(header) and os.path.getmtime(header) >= max(os.path.getmtime(asm), assembler_mtime):
        continue
    print("sxplug: %s -> %s" % (os.path.relpath(asm, project), os.path.basename(header)))
    r = subprocess.run([env.subst("$PYTHONEXE"), "-c", BUILD, asm, stem], cwd=tools, capture_output=True, text=True)
    sys.stdout.write(r.stdout)
    sys.stderr.write(r.stderr)
    if r.returncode != 0:
        sys.stderr.write("sxplug: failed to build %s\n" % os.path.relpath(asm, project))
        env.Exit(1)
