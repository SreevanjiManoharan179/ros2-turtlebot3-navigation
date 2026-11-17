"""Small helpers for readable, coloured terminal output."""
import os
import sys

_USE_COLOR = (sys.stdout.isatty() or os.environ.get('FORCE_COLOR')) and not os.environ.get('NO_COLOR')

_CODES = {
    'bold': '1', 'dim': '2', 'red': '31', 'green': '32', 'yellow': '33',
    'blue': '34', 'magenta': '35', 'cyan': '36', 'grey': '90',
}


def style(text, *names):
    if not _USE_COLOR or not names:
        return str(text)
    codes = ';'.join(_CODES[n] for n in names if n in _CODES)
    return f'\033[{codes}m{text}\033[0m'


def say(text='', *names):
    print(style(text, *names), flush=True)


def banner(title, subtitle=None, color='cyan'):
    width = max(64, len(title) + 6)
    say('=' * width, color)
    say(f'  {title}', color, 'bold')
    if subtitle:
        say(f'  {subtitle}', 'grey')
    say('=' * width, color)


def fmt(value, digits=2, unit=''):
    if value is None:
        return '-'
    if isinstance(value, float):
        return f'{value:.{digits}f}{unit}'
    return f'{value}{unit}'


def table(headers, rows, aligns=None):
    """Return a plain-text table (list of lines)."""
    cols = len(headers)
    aligns = aligns or ['<'] + ['>'] * (cols - 1)
    widths = [len(str(h)) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))

    def line(cells):
        return ' | '.join(f'{str(c):{aligns[i]}{widths[i]}}' for i, c in enumerate(cells))

    sep = '-+-'.join('-' * w for w in widths)
    return [line(headers), sep] + [line(r) for r in rows]
