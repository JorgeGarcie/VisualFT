#!/usr/bin/env python3
"""Lint source files against golden principles.

Usage:
    python3 scripts/lint_principles.py [--strict]
    --strict: exit 1 if any violations found (for CI)
"""

import argparse
import ast
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "ros2_ws" / "src"
INTERVIEW = ROOT / "interview_demos"

# Files/dirs to skip
SKIP_DIRS = {"__pycache__", "build", "install", "log", ".git", "node_modules", "check"}
SKIP_FILES = {"config.py", "constants.py", "setup.py", "setup.cfg"}

# --- GP1: One fact, one place — magic numbers ---
# Numbers that likely should be in config (skip 0, 1, -1, 2, common small ints)
MAGIC_NUMBER_RE = re.compile(
    r"(?<!\w)"           # not preceded by word char
    r"(\d+\.\d+)"        # float literal
    r"(?!\w)"             # not followed by word char
)
KNOWN_SAFE_FLOATS = {
    "0.0", "1.0", "-1.0", "0.5", "2.0", "0.1",  # common constants
    "0.001", "0.01", "1e-6", "1e-8",              # epsilon values
    "3.14159", "6.28318",                          # pi
}

# --- GP2: Centralize — duplicated quaternion conversion patterns ---
QUAT_CONV_RE = re.compile(
    r"(q\[3\],\s*q\[0\],\s*q\[1\],\s*q\[2\]|"   # RDK reorder
    r"p\[4\],\s*p\[5\],\s*p\[6\],\s*p\[3\]|"     # tcp_pose reorder
    r"as_quat\(\).*\[3\])",                         # scipy quat w extraction
    re.DOTALL,
)

# --- GP4: Fail loud — bare except / pass ---
BARE_EXCEPT_RE = re.compile(r"except\s*:")
EXCEPT_PASS_RE = re.compile(r"except(?!\s+(?:KeyboardInterrupt|SystemExit)).*:\s*\n\s*pass")


def iter_python_files(*roots):
    """Yield .py files under given roots, respecting skip rules."""
    for root in roots:
        if not root.exists():
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
            for f in filenames:
                if f.endswith(".py") and f not in SKIP_FILES:
                    yield Path(dirpath) / f


def check_noqa(line):
    """Return True if line has # noqa comment."""
    return "# noqa" in line


def check_magic_numbers(filepath, lines):
    """GP1: Flag float literals that look like magic numbers."""
    violations = []
    in_docstring = False
    for i, line in enumerate(lines, 1):
        stripped = line.strip()
        if stripped.startswith('"""') or stripped.startswith("'''"):
            if stripped.count('"""') == 1 or stripped.count("'''") == 1:
                in_docstring = not in_docstring
            continue
        if in_docstring:
            continue
        if stripped.startswith("#"):
            continue
        if check_noqa(line):
            continue
        # Skip imports, decorators, type hints
        if stripped.startswith(("import ", "from ", "@", "def ", "class ")):
            continue
        # Skip default parameter declarations (those are fine)
        if "declare_parameter" in line or "get_parameter" in line:
            continue
        for m in MAGIC_NUMBER_RE.finditer(line):
            val = m.group(1)
            if val in KNOWN_SAFE_FLOATS:
                continue
            # Skip version strings
            if "version" in line.lower():
                continue
            violations.append((filepath, i, f"Magic number {val} — consider moving to config"))
    return violations


def check_quat_conversion(filepath, lines):
    """GP2: Flag inline quaternion reordering (should be a shared utility)."""
    violations = []
    full_text = "".join(lines)
    for m in QUAT_CONV_RE.finditer(full_text):
        # Find line number
        pos = m.start()
        lineno = full_text[:pos].count("\n") + 1
        line = lines[lineno - 1] if lineno <= len(lines) else ""
        if check_noqa(line):
            continue
        violations.append((filepath, lineno, "Inline quaternion reorder — use shared utility"))
    return violations


def check_bare_except(filepath, lines):
    """GP4: Flag bare except: clauses and except-pass patterns."""
    violations = []
    for i, line in enumerate(lines, 1):
        if check_noqa(line):
            continue
        if BARE_EXCEPT_RE.search(line):
            violations.append((filepath, i, "Bare 'except:' — catch specific exceptions"))
    # Check except-pass (multi-line)
    full_text = "".join(lines)
    for m in EXCEPT_PASS_RE.finditer(full_text):
        pos = m.start()
        lineno = full_text[:pos].count("\n") + 1
        line = lines[lineno - 1] if lineno <= len(lines) else ""
        if check_noqa(line):
            continue
        violations.append((filepath, lineno, "except-pass swallows errors — log or re-raise"))
    return violations


def check_unused_imports(filepath, lines):
    """GP2: Flag obviously unused imports using AST."""
    violations = []
    source = "".join(lines)
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return violations

    imported_names = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                name = alias.asname or alias.name.split(".")[0]
                imported_names[name] = node.lineno
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name == "*":
                    continue
                name = alias.asname or alias.name
                imported_names[name] = node.lineno

    # Check if imported names appear elsewhere in source
    for name, lineno in imported_names.items():
        # Count occurrences (excluding the import line itself)
        pattern = re.compile(r"\b" + re.escape(name) + r"\b")
        count = 0
        for i, line in enumerate(lines, 1):
            if i == lineno:
                continue
            if pattern.search(line):
                count += 1
        if count == 0:
            line = lines[lineno - 1] if lineno <= len(lines) else ""
            if not check_noqa(line):
                violations.append((filepath, lineno, f"Unused import '{name}' — remove it"))
    return violations


def main():
    parser = argparse.ArgumentParser(description="Lint VisualFT against golden principles")
    parser.add_argument("--strict", action="store_true", help="Exit 1 on violations")
    args = parser.parse_args()

    all_violations = defaultdict(list)

    for fp in iter_python_files(SRC, INTERVIEW, ROOT / "scripts"):
        try:
            lines = fp.read_text().splitlines(keepends=True)
        except Exception:
            continue

        rel = fp.relative_to(ROOT)

        v = check_magic_numbers(rel, lines)
        all_violations["GP1: One fact, one place"].extend(v)

        v = check_quat_conversion(rel, lines)
        all_violations["GP2: Centralize, don't duplicate"].extend(v)

        v = check_bare_except(rel, lines)
        all_violations["GP4: Fail loud, not silent"].extend(v)

        v = check_unused_imports(rel, lines)
        all_violations["GP2: Centralize, don't duplicate"].extend(v)

    total = sum(len(v) for v in all_violations.values())
    print(f"\n{'='*60}")
    print(f"  VisualFT Lint Report — {total} violation(s)")
    print(f"{'='*60}\n")

    if total == 0:
        print("  All clean!")
        return 0

    for principle, violations in sorted(all_violations.items()):
        if not violations:
            continue
        print(f"\n## {principle} ({len(violations)} violations)\n")
        for filepath, line, msg in sorted(violations):
            print(f"  {filepath}:{line} — {msg}")
            # FIX instruction for agents
            print(f"    FIX: {msg.split('—')[-1].strip() if '—' in msg else 'Address the issue'}")
        print()

    if args.strict:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
