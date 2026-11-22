"""The mazes a robot can be run in by name (--maze NAME): ratasim/mazes's and the competition mazes of
github.com/micromouseonline/mazefiles (the submodule external/mazefiles: 525 classic 16x16 from APEC, All Japan, UK,
..., 16 training, 42 half size). Each is a text file; the scene is built from it when a robot runs, nothing else is
stored.

    ratasim mazes                    # every name, by set
    ratasim mazes japan              # the names with "japan" in them (a glob if it has * ? [)
    ratasim mazes --show japan2000   # one maze drawn, its size and its file
    ratasim mazes --paths uk20       # their files, one per line (for ratasim batch OUT $(ratasim mazes --paths ...))
"""
import argparse
import fnmatch
import os
import shutil

from . import paths
from .maze import MAZES, Maze, maze_path


def sets():
    """[(set name, its folder, [maze names])]: the bundled ones first, then the submodule's folders."""
    folders = [('ratasim/mazes', MAZES)] + [(s, os.path.join(paths.MAZEFILES, s)) for s in paths.MAZE_SETS]
    return [(s, f, sorted(n[:-4] for n in os.listdir(f) if n.endswith('.txt'))) for s, f in folders if os.path.isdir(f)]


def matches(name, pattern):
    if not pattern:
        return True
    if any(c in pattern for c in '*?['):
        return fnmatch.fnmatch(name, pattern)
    return pattern.lower() in name.lower()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('pattern', nargs='?', help='part of a name, or a glob')
    p.add_argument('--show', metavar='NAME', help='draw that maze')
    p.add_argument('--paths', action='store_true', help='print the matching files, one per line')
    a = p.parse_args()
    if a.show:
        path = maze_path(a.show)
        m = Maze.load(path)
        print(f'{a.show}: {m.size}x{m.size}, {os.path.relpath(path, paths.REPO)}')
        print(m.text(), end='')
        return
    if paths._empty('external/mazefiles'):
        print('(external/mazefiles is not downloaded: only the bundled mazes. '
              'Download it with: git submodule update --init external/mazefiles)\n')
    found = [(s, f, [n for n in ns if matches(n, a.pattern)]) for s, f, ns in sets()]
    if a.paths:
        for _, f, ns in found:
            for n in ns:
                print(os.path.relpath(os.path.join(f, n + '.txt')))
        return
    width = shutil.get_terminal_size((100, 20)).columns
    for s, _, ns in found:
        if not ns:
            continue
        print(f'{s} ({len(ns)}):')
        col = max(len(n) for n in ns) + 2
        per = max(1, (width - 2) // col)
        for i in range(0, len(ns), per):
            print('  ' + ''.join(n.ljust(col) for n in ns[i:i + per]).rstrip())
    if not any(ns for _, _, ns in found):
        print(f'no maze matches {a.pattern!r}')


if __name__ == '__main__':
    main()
