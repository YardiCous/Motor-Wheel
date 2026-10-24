"""Apply clang-format, then the two BARR-C rules it cannot express."""
from pathlib import Path
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def sources():
    return sorted(ROOT.joinpath("app_program").rglob("*.[ch]")) + sorted(
        ROOT.joinpath("testing/host").glob("*.[ch]")) + [
            ROOT / "lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c",
            ROOT / "lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h",
            ROOT / "config/config_device.h"]


def logical_parentheses(text):
    """Parenthesize nontrivial logical operands without changing precedence."""
    lexer = re.compile(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|'
        r"'(?:\\.|[^'\\])*'|[A-Za-z_]\w*|"
        r'0[xX][\da-fA-F]+[uUlL]*|\d+[uUlL]*|'
        r'&&|\|\||->|==|!=|<=|>=|<<|>>|\+=|-=|\|=|&=|[^\s]', re.S)
    precedence = {"||": 1, "&&": 2}
    boundaries = {";", ",", "=", "?", ":", "{", "}", "return", "case",
                  "+=", "-=", "|=", "&="}
    for _ in range(10000):
        tokens = [match for match in lexer.finditer(text)
                  if not match[0].startswith(("/*", "//"))]
        change = None
        for position, operator in enumerate(tokens):
            if operator[0] not in precedence:
                continue
            level = precedence[operator[0]]
            for direction in (-1, 1):
                cursor = position + direction
                depth = 0
                span = []
                while 0 <= cursor < len(tokens):
                    value = tokens[cursor][0]
                    opening, closing = ((")", "(") if direction < 0
                                        else ("(", ")"))
                    if value == opening:
                        depth += 1
                    elif value == closing:
                        if depth == 0:
                            break
                        depth -= 1
                    if depth == 0 and (value in boundaries or
                            (value in precedence and precedence[value] <= level)):
                        break
                    span.append(cursor)
                    cursor += direction
                if not span:
                    continue
                first, last = min(span), max(span)
                content = [match[0] for match in tokens[first:last + 1]]
                if len(content) == 1:
                    continue
                enclosed = False
                if content[0] == "(" and content[-1] == ")":
                    depth = 0
                    enclosed = True
                    for value in content[:-1]:
                        depth += (value == "(") - (value == ")")
                        if depth == 0:
                            enclosed = False
                            break
                if not enclosed:
                    change = (tokens[first].start(), tokens[last].end())
                    break
            if change:
                break
        if not change:
            return text
        begin, end = change
        text = text[:begin] + "(" + text[begin:end] + ")" + text[end:]
    raise ValueError("logical operand formatting did not converge")

def main():
    formatter = os.environ.get("CLANG_FORMAT") or shutil.which("clang-format")
    if not formatter:
        raise SystemExit("Install clang-format or set CLANG_FORMAT to its executable")
    for path in sources():
        path.write_text(logical_parentheses(path.read_text()))
    subprocess.run([formatter, "-i", *map(str, sources())], check=True)
    for path in sources():
        text = path.read_text()
        text = re.sub(
            r"^((?:static\s+)?\w+\s+)([a-z][a-z_0-9]*)\(([^;{}]*)\)\n\{",
            r"\1\2 (\3)\n{", text, flags=re.M)
        text = re.sub(r"^([ ]{4,})break;", lambda match:
                      " " * (len(match[1]) - 4) + "break;", text, flags=re.M)
        path.write_text(text.rstrip() + "\n\n")


if __name__ == "__main__":
    main()
