#!/usr/bin/env python3
"""Check documentation health: broken references, stale docs.

Usage:
    python3 scripts/check_docs_freshness.py [--strict]
    --strict: exit 1 if any issues found
"""

import argparse
import re
import sys
from datetime import datetime, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKIP_DIRS = {"__pycache__", "build", "install", "log", ".git", "node_modules"}

# Library prefixes to skip (not file paths)
LIBRARY_PREFIXES = (
    "torch.", "np.", "cv2.", "rclpy.", "scipy.", "PIL.",
    "flexivrdk.", "onnxruntime.", "matplotlib.", "serial.",
    "std_msgs.", "geometry_msgs.", "sensor_msgs.", "std_srvs.",
)


def iter_markdown_files():
    """Yield all .md files under ROOT."""
    for dirpath, dirnames, filenames in (ROOT).walk() if hasattr(ROOT, "walk") else _walk(ROOT):
        dirnames_list = list(dirpath.iterdir()) if False else []  # handled below
        for f in filenames:
            if f.endswith(".md"):
                yield dirpath / f


def _walk(root):
    """os.walk wrapper returning Path objects."""
    import os
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        yield Path(dirpath), dirnames, filenames


def check_backtick_paths(filepath, content):
    """Find backtick-wrapped paths with / and verify they exist."""
    issues = []
    # Match `path/to/something` or `path/to/something.ext`
    pattern = re.compile(r"`([^`\s]+/[^`\s]+)`")
    for i, line in enumerate(content.splitlines(), 1):
        for m in pattern.finditer(line):
            ref = m.group(1)
            # Skip library references
            if any(ref.startswith(p) for p in LIBRARY_PREFIXES):
                continue
            # Skip URLs
            if ref.startswith(("http://", "https://", "//", "ftp://")):
                continue
            # Skip shell commands that look like paths
            if ref.startswith(("-", "--", "$")):
                continue
            # Resolve relative to ROOT
            target = ROOT / ref
            if not target.exists():
                # Also try relative to file's directory
                target2 = filepath.parent / ref
                if not target2.exists():
                    issues.append((filepath, i, f"Broken reference: `{ref}` — file not found"))
    return issues


def check_staleness(filepath, content):
    """Flag docs with old 'Last updated' dates or pending markers."""
    issues = []
    # Check for "Last updated: YYYY-MM-DD" older than 30 days
    date_pattern = re.compile(r"[Ll]ast [Uu]pdated:?\s*(\d{4}-\d{2}-\d{2})")
    for i, line in enumerate(content.splitlines(), 1):
        m = date_pattern.search(line)
        if m:
            try:
                doc_date = datetime.strptime(m.group(1), "%Y-%m-%d")
                if datetime.now() - doc_date > timedelta(days=30):
                    days = (datetime.now() - doc_date).days
                    issues.append((filepath, i, f"Stale: last updated {days} days ago"))
            except ValueError:
                pass

    # Check for pending/awaiting markers
    pending_pattern = re.compile(r"\b(Pending|Awaiting|TODO|FIXME|TBD)\b", re.IGNORECASE)
    for i, line in enumerate(content.splitlines(), 1):
        if pending_pattern.search(line):
            # Skip if in a code block
            if line.strip().startswith(("```", "#", "- [x]")):
                continue
            issues.append((filepath, i, f"Pending marker: '{pending_pattern.search(line).group()}'"))
    return issues


def check_claude_md_table(content):
    """Verify Deeper Context table in CLAUDE.md references existing files."""
    issues = []
    claude_md = ROOT / "CLAUDE.md"
    if not claude_md.exists():
        return [("(root)", 0, "CLAUDE.md not found")]

    in_table = False
    for i, line in enumerate(content.splitlines(), 1):
        if "Deeper Context" in line:
            in_table = True
            continue
        if in_table:
            if line.strip() == "" or line.startswith("#"):
                in_table = False
                continue
            # Extract path from table row: | topic | `path` |
            path_match = re.search(r"`([^`]+)`", line)
            if path_match:
                ref = path_match.group(1)
                target = ROOT / ref
                if not target.exists():
                    issues.append((claude_md, i, f"Deeper Context broken: `{ref}` not found"))
    return issues


def main():
    parser = argparse.ArgumentParser(description="Check VisualFT docs freshness")
    parser.add_argument("--strict", action="store_true", help="Exit 1 on issues")
    args = parser.parse_args()

    all_issues = []

    for fp in _walk_md(ROOT):
        try:
            content = fp.read_text()
        except Exception:
            continue

        all_issues.extend(check_backtick_paths(fp, content))
        all_issues.extend(check_staleness(fp, content))

    # Special CLAUDE.md check
    claude_md = ROOT / "CLAUDE.md"
    if claude_md.exists():
        all_issues.extend(check_claude_md_table(claude_md.read_text()))

    print(f"\n{'='*60}")
    print(f"  Doc Freshness Report — {len(all_issues)} issue(s)")
    print(f"{'='*60}\n")

    if not all_issues:
        print("  All docs healthy!")
        return 0

    for filepath, line, msg in sorted(all_issues):
        rel = filepath.relative_to(ROOT) if isinstance(filepath, Path) and filepath.is_relative_to(ROOT) else filepath
        print(f"  {rel}:{line} — {msg}")

    if args.strict:
        return 1
    return 0


def _walk_md(root):
    """Yield all .md files."""
    import os
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for f in filenames:
            if f.endswith(".md"):
                yield Path(dirpath) / f


if __name__ == "__main__":
    sys.exit(main())
