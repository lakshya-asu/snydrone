"""Build a single-file, dependency-free copy of the demo.

The demo imports shotlib.mjs as a module, which needs an HTTP server.
This inlines the module so the page opens from a file:// URL, an email
attachment, or any static host, with no build tooling and no server.

Inlined AT BUILD TIME rather than copy-pasted on purpose: shotlib.mjs
stays the single source of truth, so the standalone copy cannot quietly
drift from the tested module the way a fork would.

    python build_standalone.py            # writes standalone.html
    python build_standalone.py --check    # verify it is up to date, exit 1 if not
"""

import argparse
import pathlib
import sys

HERE = pathlib.Path(__file__).parent
SRC = HERE / "index.html"
LIB = HERE / "shotlib.mjs"
OUT = HERE / "standalone.html"

IMPORT_LINE = ('import { parseShotSpec, sampleTrajectory, checkTrajectory, '
               'describeViolation, worstByKind, ShotSpecError } '
               'from "./shotlib.mjs";')

BANNER = """// --- inlined from shotlib.mjs by build_standalone.py, do not edit here ---
"""


def build():
    html = SRC.read_text(encoding="utf-8")
    lib = LIB.read_text(encoding="utf-8")

    if IMPORT_LINE not in html:
        raise SystemExit(
            "index.html no longer contains the expected import line; "
            "update IMPORT_LINE in this script rather than hand-editing "
            "the generated file.")

    # Strip the export keywords: once inlined, everything shares one scope.
    inlined = BANNER + lib.replace("export class ", "class ") \
                          .replace("export const ", "const ") \
                          .replace("export function ", "function ")
    out = html.replace(IMPORT_LINE, inlined)

    # With nothing left to import, drop type="module" too. A module script
    # is blocked wherever the document has an opaque origin (a data: URL,
    # some email and chat previews), and gains nothing here. The script tag
    # is the last element in the body and every node it queries is above
    # it, so losing the module's implicit defer is safe.
    if '<script type="module">' not in out:
        raise SystemExit("expected a single module script tag in index.html")
    return out.replace('<script type="module">', "<script>")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="fail if standalone.html is stale")
    args = ap.parse_args()

    built = build()
    if args.check:
        current = OUT.read_text(encoding="utf-8") if OUT.exists() else ""
        if current != built:
            print("standalone.html is stale; run build_standalone.py")
            return 1
        print("standalone.html is up to date")
        return 0

    OUT.write_text(built, encoding="utf-8")
    print(f"wrote {OUT} ({len(built):,} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
