#!/usr/bin/env python3
"""Audit runtime constraints; vendored JS is excluded from the literal audit.

The development-only JavaScript lexer uses Python's standard library. Browser
and server runtime dependency requirements are unchanged.
"""
import ast
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
# Internal schema keys are not external robot/topic/service/action names.
INTERNAL_KEYS = {'limo_status_template', 'limo_status', 'limo_ips', 'limo_battery_v',
                 'limo_at', 'limo_area', 'spare_prefix', 'spare_unarmed', 'spare_area'}
LITERAL_PATTERNS = [r'cf23\w*', r'/preflight(?:/|$)', r'/coshow(?:/|$)',
                    r'/(?:land|arm)(?:/|$)', r'navigate_to_pose', r'/aideck(?:/|$)',
                    r'marker_detections', r'\blimo_[A-Za-z0-9_]*', r'\bspare_[A-Za-z0-9_]*']


def assert_string_external_names_absent(value, path, line):
    for pattern in LITERAL_PATTERNS:
        for match in re.finditer(pattern, value):
            assert match.group() in INTERNAL_KEYS, (path, line, pattern, match.group())


def assert_external_names_absent(tree, path):
    for node in ast.walk(tree):
        if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
            continue
        assert_string_external_names_absent(node.value, path, node.lineno)


JS_NUMBER = re.compile(r'(?:0[xX][\da-fA-F_]+|0[bB][01_]+|0[oO][0-7_]+|'
                       r'(?:\d[\d_]*(?:\.[\d_]*)?|\.\d[\d_]*)(?:[eE][+-]?[\d_]+)?)(?:n)?')
JS_WORD = re.compile(r'[$\w]+', re.UNICODE)
JS_OPERATORS = re.compile(r'===|!==|>>>|\*\*=|=>|==|!=|<=|>=|&&|\|\||\?\?|\?\.|'
                          r'\+\+|--|\*\*|<<|>>|[+*/%&|^!-]=|.')


def javascript_tokens(source, path):
    """Yield (kind, decoded value, line), excluding comments and regex bodies.

    Strings decode JS escapes, and templates recursively lex ${expressions}.
    The expression-start state distinguishes regex literals from division;
    control-statement parentheses allow a regex immediately after `if (...)`.
    Malformed/unterminated literals fail closed instead of hiding following code.
    This is a lexer, not a syntax validator; Node --check owns syntax validation.
    """
    index, line = 0, 1

    def advance(count=1):
        nonlocal index, line
        line += source[index:index + count].count('\n')
        index += count

    def escape():
        advance()  # backslash
        assert index < len(source), (path, line, 'unfinished JavaScript escape')
        char = source[index]
        advance()
        if char in '\r\n':
            if char == '\r' and source[index:index + 1] == '\n':
                advance()
            return ''
        if char == 'u' and source[index:index + 1] == '{':
            end = source.find('}', index + 1)
            assert end >= 0, (path, line, 'unfinished Unicode escape')
            digits = source[index + 1:end]
            assert re.fullmatch(r'[\da-fA-F]{1,6}', digits), (path, line, 'invalid Unicode escape')
            advance(end + 1 - index)
            return chr(int(digits, 16))
        if char in ('u', 'x'):
            length = 4 if char == 'u' else 2
            digits = source[index:index + length]
            assert re.fullmatch(r'[\da-fA-F]{' + str(length) + '}', digits), (path, line, 'invalid escape')
            advance(length)
            return chr(int(digits, 16))
        return {'n': '\n', 'r': '\r', 't': '\t', 'b': '\b', 'f': '\f',
                'v': '\v', '0': '\0'}.get(char, char)

    def string(quote):
        start, value = line, []
        advance()
        while index < len(source):
            char = source[index]
            if char == quote:
                advance()
                yield 'string', ''.join(value), start
                return
            if char == '\\':
                value.append(escape())
            elif quote == '`' and source[index:index + 2] == '${':
                yield 'string', ''.join(value), start
                value = []
                advance(2)
                yield from code(template_expression=True)
                start = line
            else:
                assert quote == '`' or char not in '\r\n', (path, line, 'newline in JavaScript string')
                value.append(char)
                advance()
        raise AssertionError((path, start, 'unterminated JavaScript string/template'))

    def regex():
        start, in_class = line, False
        advance()
        while index < len(source):
            char = source[index]
            assert char not in '\r\n', (path, start, 'unterminated JavaScript regex')
            if char == '\\':
                advance(2)
            elif char == '/' and not in_class:
                advance()
                flags = JS_WORD.match(source, index)
                if flags:
                    advance(len(flags.group()))
                return
            else:
                if char == '[':
                    in_class = True
                elif char == ']':
                    in_class = False
                advance()
        raise AssertionError((path, start, 'unterminated JavaScript regex'))

    def code(template_expression=False):
        depth, expression_start, previous, parens = 0, True, '', []
        while index < len(source):
            char, start = source[index], line
            if char.isspace():
                advance()
                continue
            if source[index:index + 2] == '//':
                end = source.find('\n', index)
                advance((len(source) if end < 0 else end) - index)
                continue
            if source[index:index + 2] == '/*':
                end = source.find('*/', index + 2)
                assert end >= 0, (path, start, 'unterminated JavaScript comment')
                advance(end + 2 - index)
                continue
            if char == '}' and template_expression and depth == 0:
                advance()
                return
            if char in ('"', "'", '`'):
                yield from string(char)
                expression_start, previous = False, 'literal'
                continue
            if char == '/' and expression_start:
                regex()
                expression_start, previous = False, 'literal'
                continue
            number = JS_NUMBER.match(source, index)
            if number:
                raw = number.group().replace('_', '').removesuffix('n')
                value = int(raw, 0) if raw.lower().startswith(('0x', '0b', '0o')) else float(raw)
                advance(len(number.group()))
                yield 'number', value, start
                expression_start, previous = False, 'literal'
                continue
            word = JS_WORD.match(source, index)
            if word:
                value = word.group()
                advance(len(value))
                yield 'word', value, start
                expression_start = value in {'return', 'throw', 'case', 'delete', 'void', 'typeof',
                                              'new', 'in', 'of', 'instanceof', 'yield', 'await', 'else', 'do'}
                previous = value
                continue
            value = JS_OPERATORS.match(source, index).group()
            advance(len(value))
            yield 'punct', value, start
            if value == '(':
                parens.append(previous in {'if', 'while', 'for', 'with', 'switch', 'catch'})
            if value == '{':
                depth += 1
            elif value == '}':
                depth -= 1
            if value == ')':
                expression_start = parens.pop() if parens else False
            else:
                expression_start = value not in (']', '.', '?.', '++', '--')
            previous = value
        assert not template_expression, (path, line, 'unterminated template expression')

    yield from code()


def assert_javascript_literals_allowed(source, path):
    tokens = list(javascript_tokens(source, path))
    strings = [token for token in tokens if token[0] == 'string']
    numbers = [token for token in tokens if token[0] == 'number']
    for _, value, line in strings:
        assert_string_external_names_absent(value, path, line)

    # Detect the reviewed marker-range hole without banning typography/geometry
    # values 11 and 14. Only marker-named arrays or comparisons of id/marker
    # operands are schema constants. Numeric spelling (hex/exponent/etc.) cannot
    # bypass the rule. Semicolons keep unrelated statements from being combined.
    statement, bounds = [], {}
    for token in tokens + [('punct', ';', 0)]:
        if token[1] != ';':
            statement.append(token)
            continue
        marker_context = any(kind in ('word', 'string') and
                             re.search(r'marker.*(?:ids|range)|(?:ids|range).*marker', str(value), re.I)
                             for kind, value, _ in statement)
        values = {value for kind, value, _ in statement if kind == 'number'}
        if marker_context and any(value == '[' for _, value, _ in statement):
            assert not {11, 14} <= values, (path, token[2], 'hardcoded mission marker range 11..14')
        for pos, (kind, value, line) in enumerate(statement):
            if kind != 'number' or value not in (11, 14):
                continue
            for direction in (-1, 1):
                op, operand = pos + direction, pos + 2 * direction
                if not (0 <= operand < len(statement) and 0 <= op < len(statement)):
                    continue
                operand_kind, operand_value, _ = statement[operand]
                if statement[op][1] not in ('<', '<=', '>', '>=') or operand_kind != 'word':
                    continue
                if not re.search(r'(?:^id$|_id$|Id$|marker)', operand_value, re.IGNORECASE):
                    continue
                bounds.setdefault(operand_value, set()).add(value)
                assert bounds[operand_value] != {11, 14}, (path, line, 'hardcoded mission marker range 11..14')
        statement, bounds = [], {}
    return len(strings), len(numbers)



def main():
    print('Python:', sys.version.split()[0])
    imports = set()
    paths = sorted(ROOT.glob('*.py'))
    for path in paths:
        text = path.read_text()
        tree = ast.parse(text, feature_version=(3, 10))
        assert_external_names_absent(tree, path)
        assert not re.search(r'use_sim_time|tomllib|TaskGroup|asyncio\.timeout', text), path
        assert not any(isinstance(n, ast.Match) for n in ast.walk(tree)), path
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split('.')[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split('.')[0])
            annotation = getattr(node, 'annotation', None)
            if annotation is not None:
                assert not any(isinstance(n, ast.BinOp) and isinstance(n.op, ast.BitOr)
                               for n in ast.walk(annotation)), path
    extras = imports - set(sys.stdlib_module_names) - {
        'dashboard', 'aiohttp', 'yaml', 'rclpy', 'rosidl_runtime_py'}
    assert not extras, extras
    static_root = ROOT / 'static'
    static = list(static_root.rglob('*'))
    javascript = sorted(path for path in static if path.suffix == '.js')
    vendor = [path for path in javascript if 'vendor' in path.relative_to(static_root).parts]
    authored = [path for path in javascript if path not in vendor]
    string_count, number_count = 0, 0
    for path in authored:
        strings, numbers = assert_javascript_literals_allowed(path.read_text(), path)
        string_count += strings
        number_count += numbers
    for path in static:
        if path.suffix in ('.html', '.js', '.css'):
            # Vendor licenses/documentation contain URLs, so check executable
            # dependency/request syntax. Browser network QA verifies actual I/O.
            requests = r"(?:\bfrom\s*|\bimport\s*\(?|\bfetch\s*\(|\b(?:src|href)\s*=)\s*['\"](?:https?:)?//|url\s*\(\s*['\"]?(?:https?:)?//"
            assert not re.search(requests, path.read_text()), path
    assert not any('socket' == name for name in imports), 'Runtime must not open UDP sockets'
    print('Runtime imports:', ', '.join(sorted(imports)))
    print('PASS: {} runtime files, Python 3.10 grammar, no match/union/newer stdlib'.format(len(paths)))
    print('External literal patterns:', LITERAL_PATTERNS)
    print('Internal schema-key allowlist:', sorted(INTERNAL_KEYS))
    print('PASS: every Python runtime string literal tested; no external-name literal matched outside schema keys')
    print('PASS: {} authored JavaScript files, {} string literals, {} number literals'.format(
        len(authored), string_count, number_count))
    print('PASS: no authored JavaScript external-name literals or hardcoded mission marker range 11..14')
    print('{} vendor JavaScript files excluded from external-name/literal audit'.format(len(vendor)))
    print('PASS: only aiohttp/PyYAML + stdlib/ROS; no socket/UDP module')
    print('PASS: no external dependency/request syntax in static HTML/JS/CSS; vendor comment URLs are allowed')


if __name__ == '__main__':
    main()
