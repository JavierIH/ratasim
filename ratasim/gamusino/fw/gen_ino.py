"""A sketch (.ino) to C++, as the Arduino IDE does before compiling it: Arduino.h first, a prototype for each of its
functions (it calls some before defining them), then the sketch, its lines numbered as in the .ino.

    python3 gen_ino.py SKETCH.ino OUT.cpp
"""
import re
import sys

# A function's definition at the top level: a type, a name, its parameters, its opening brace (on that line or the next).
DEF = re.compile(r'^([A-Za-z_][\w:<>\s\*&]*?[\s\*&])([A-Za-z_]\w*)\s*\(([^;{)]*)\)\s*(\{|$)', re.M)
KEYWORDS = {'if', 'else', 'for', 'while', 'switch', 'return', 'do'}


def strip_comments(src):
    out = re.sub(r'/\*.*?\*/', lambda m: re.sub(r'[^\n]', ' ', m.group()), src, flags=re.S)
    return re.sub(r'//[^\n]*', '', out)


def prototypes(src):
    clean, depth, top = strip_comments(src), 0, []
    for line in clean.split('\n'):         # only lines at brace depth 0
        top.append(line if depth == 0 else '')
        depth += line.count('{') - line.count('}')
    text, out = '\n'.join(top), []
    for m in DEF.finditer(text):
        ret, name, params = m.group(1).strip(), m.group(2), m.group(3).strip()
        if name in KEYWORDS or ret.split()[-1] in KEYWORDS or ret.startswith(('return', 'else')):
            continue
        if m.group(4) == '' and not text[m.end():].lstrip().startswith('{'):
            continue
        params = re.sub(r'\s*=\s*[^,]+', '', params)    # default arguments stay in the definition
        out.append(f'{ret} {name}({params});')
    return out


def main(src_path, out_path):
    src = open(src_path, encoding='utf-8').read()
    protos = prototypes(src)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('#include <Arduino.h>\n')
        # its prototypes after its own #includes and types: at the first function, as the IDE puts them
        first = min((m.start() for m in DEF.finditer(src) if m.group(2) not in KEYWORDS), default=0)
        head_lines = src[:first].count('\n')
        lines = src.split('\n')
        f.write(f'#line 1 "{src_path}"\n')
        f.write('\n'.join(lines[:head_lines]) + '\n')
        f.write('\n'.join(protos) + '\n')
        f.write(f'#line {head_lines + 1} "{src_path}"\n')
        f.write('\n'.join(lines[head_lines:]))


if __name__ == '__main__':
    main(*sys.argv[1:3])
