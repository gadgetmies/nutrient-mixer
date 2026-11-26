#!/usr/bin/env python3
"""Turns an Arduino sketch (.ino) into a C++ file the way the Arduino builder does.

The Arduino builder adds `#include <Arduino.h>` and prototypes for the functions of the sketch, so that functions
can be called before they are defined. The prototypes are inserted before the first function definition (the
types they use are declared above it). Default arguments are left out of the prototypes, as they may only be
given once. `#line` directives keep compiler messages pointing at the original sketch.

Usage: gen_prototypes.py sketch.ino output.cpp
"""
import re
import sys


def strip_code(text):
    """Returns the text with comments, strings and character literals replaced by spaces (same offsets)."""
    out = list(text)
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            for k in range(i, j):
                out[k] = " "
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                if out[k] != "\n":
                    out[k] = " "
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            for k in range(i + 1, min(j, n)):
                if out[k] != "\n":
                    out[k] = " "
            i = j + 1
        else:
            i += 1
    return "".join(out)


def remove_default_arguments(parameters):
    result = []
    depth = 0
    skipping = False
    for c in parameters:
        if c in "(<[{":
            depth += 1
        elif c in ")>]}":
            depth -= 1
        if c == "=" and depth == 0:
            skipping = True
            continue
        if c == "," and depth == 0:
            skipping = False
        if not skipping:
            result.append(c)
    return re.sub(r"\s+", " ", "".join(result)).strip()


DEFINITION = re.compile(
    r"(?P<signature>(?:[A-Za-z_][\w:<>,\*&\s]*?[\s\*&])"  # return type and qualifiers
    r"(?P<name>[A-Za-z_]\w*)\s*\((?P<parameters>[^;{}()]*(?:\([^()]*\)[^;{}()]*)*)\))\s*(?:const\s*)?\{"
)
KEYWORDS = {"if", "for", "while", "switch", "return", "else", "do", "case", "sizeof", "catch"}


def find_functions(code):
    """Yields (start offset, prototype) for function definitions at file scope."""
    depth = 0
    statement_start = 0
    i = 0
    while i < len(code):
        c = code[i]
        if c == "{":
            if depth == 0:
                candidate = code[statement_start:i + 1]
                match = DEFINITION.search(candidate)
                if match and match.end() == len(candidate) and match.group("name") not in KEYWORDS:
                    head = candidate[:match.start()]
                    # Everything before the signature in the same statement must be whitespace or a preprocessor line
                    if head.strip() == "" or all(line.strip().startswith("#") or line.strip() == "" for line in head.splitlines()):
                        signature = match.group("signature")
                        return_type = signature[:signature.rfind(match.group("name"))]
                        if not re.search(r"\b(struct|class|enum|union|namespace)\b", return_type) and "=" not in return_type:
                            prototype = "%s%s(%s);" % (
                                re.sub(r"\s+", " ", return_type).lstrip(),
                                match.group("name"),
                                remove_default_arguments(match.group("parameters")))
                            yield statement_start + match.start(), prototype
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                statement_start = i + 1
        elif c == ";" and depth == 0:
            statement_start = i + 1
        elif c == "\n" and depth == 0 and code[statement_start:i].strip().startswith("#"):
            statement_start = i + 1
        i += 1


def main():
    source_path, output_path = sys.argv[1], sys.argv[2]
    with open(source_path, encoding="utf-8") as f:
        source = f.read()
    code = strip_code(source)
    functions = list(find_functions(code))
    if not functions:
        sys.exit("No functions found in " + source_path)

    first = functions[0][0]
    # Insert at the beginning of the line of the first definition
    insert_at = source.rfind("\n", 0, first) + 1
    line_number = source.count("\n", 0, insert_at) + 1
    prototypes = []
    seen = set()
    for _, prototype in functions:
        if prototype not in seen:
            seen.add(prototype)
            prototypes.append(prototype)

    quoted_path = source_path.replace("\\", "\\\\").replace('"', '\\"')
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("#include <Arduino.h>\n")
        f.write('#line 1 "%s"\n' % quoted_path)
        f.write(source[:insert_at])
        f.write("// Prototypes generated by gen_prototypes.py\n")
        for prototype in prototypes:
            f.write(prototype + "\n")
        f.write('#line %d "%s"\n' % (line_number, quoted_path))
        f.write(source[insert_at:])


if __name__ == "__main__":
    main()
