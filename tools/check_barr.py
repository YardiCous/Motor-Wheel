"""Check the mechanically enforceable project subset of BARR-C:2018.

This is a build gate, not a certification of the rules requiring code review.
"""
from pathlib import Path
import re
import sys

from format_c import ROOT, sources, logical_parentheses


def strip_literals(text):
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda match: "\n" * match[0].count("\n"), text,
                  flags=re.S)


def check_file(path):
    text = path.read_text()
    code = strip_literals(text)
    errors = []
    if logical_parentheses(text) != text:
        errors.append("nontrivial logical operand lacks parentheses")
    for number, line in enumerate(text.splitlines(), 1):
        if len(line) > 80:
            errors.append(f"{number}: line exceeds 80 columns")
        if "\t" in line or "\r" in line:
            errors.append(f"{number}: tab or CR character")
        if any(ord(character) < 32 for character in line):
            errors.append(f"{number}: nonprinting character")
    if not text.endswith("/*** End of file. ***/\n\n"):
        errors.append("missing end-of-file comment and blank line")
    if re.search(r'\b(auto|register|goto|continue|short|long)\b', code):
        errors.append("forbidden keyword")
    if re.search(r'\b(abort|exit|setjmp|longjmp)\s*\(', code):
        errors.append("forbidden jump function")
    if re.search(r'\bwhile\s*\(\s*(?:1|true)\s*\)', code):
        errors.append("use for (;;) for an infinite loop")
    if re.search(r'^\s*#\s*define\s+\w+\(', code, re.M):
        errors.append("function-like macro requires review; use a function")
    if path.suffix == ".h":
        if not re.search(r'^#ifndef [A-Z][A-Z_0-9]*\n#define ', text, re.M):
            errors.append("missing header guard")
    else:
        if f'#include "{path.stem}.h"' not in text:
            errors.append("missing own header")
        for match in re.finditer(
                r'^((?:static\s+)?\w+)\s+(\w+)\s*\([^;{}]*\)\n\{', code, re.M):
            name = match[2]
            end = match.end()
            depth = 1
            while depth and end < len(code):
                depth += (code[end] == "{") - (code[end] == "}")
                end += 1
            if code[match.start():end].count("\n") + 1 > 100:
                errors.append(f"{name}: function exceeds 100 lines")
            if len(name) > 31 or name.lower() != name or name.startswith("_"):
                errors.append(f"{name}: function name violates BARR-C")
            if (not match[1].startswith("static") and name not in {"main", "usermain"}
                    and not name.startswith(path.stem + "_")):
                errors.append(f"{name}: public function lacks module prefix")
    # Single-statement control bodies and same-line braces are prohibited.
    for number, line in enumerate(code.splitlines(), 1):
        if "{" in line and line.strip() != "{":
            # Initializers have their own syntax and are not block statements.
            if re.search(r'\b(if|else|for|while|switch)\b', line):
                errors.append(f"{number}: opening brace must occupy its own line")
        if re.search(r'\b(if|while|for)\s*\([^;{}]*\)\s+\w+\s*\(', line):
            errors.append(f"{number}: unbraced control statement")
    return errors


def main():
    problems = []
    modules = {}
    for path in sources():
        if path.suffix == ".c":
            first = path.stem[:8]
            if first in modules and modules[first] != path.stem:
                problems.append(f"module collision: {path.stem}, {modules[first]}")
            modules[first] = path.stem
            if not path.with_suffix(".h").exists():
                problems.append(f"{path}: source lacks its corresponding header")
        if not re.fullmatch(r'[a-z][a-z_0-9]*', path.stem):
            problems.append(f"{path}: invalid module name")
        problems.extend(f"{path.relative_to(ROOT)}:{error}" for error in check_file(path))
    if problems:
        print("\n".join(problems), file=sys.stderr)
        raise SystemExit(1)
    print(f"PASS: BARR-C automated subset ({len(sources())} files)")


if __name__ == "__main__":
    main()
