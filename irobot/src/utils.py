from __future__ import annotations

from contextlib import contextmanager


class SkipWith(Exception):
    pass


class _ColorPrint:
    _YELLOW = '\033[93m'
    _GREEN  = '\033[92m'
    _RESET  = '\033[0m'

    def print_skip(self, msg: str) -> None:
        print(f'{self._YELLOW}{msg}{self._RESET}')

    def print_run(self, msg: str) -> None:
        print(f'{self._GREEN}{msg}{self._RESET}')


@contextmanager
def skip_run(flag: str, f: str):
    """Context manager to selectively run or skip a block of code.

    Usage in main.py::

        with skip_run('run', 'my_demo') as check:
            with check():
                component = MyDemo(...)

    Change 'run' to 'skip' (or vice-versa) on one line to toggle the block.
    """

    @contextmanager
    def check_active():
        p = _ColorPrint()
        if flag == 'skip':
            p.print_skip('{:>12}  {:>2}  {:>12}'.format('Skipping the block', '|', f))
            raise SkipWith()
        else:
            p.print_run('{:>12}  {:>3}  {:>12}'.format('Running the block', '|', f))
            yield

    try:
        yield check_active
    except SkipWith:
        pass
