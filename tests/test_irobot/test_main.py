from __future__ import annotations

from irobot.main import get_hello

def it_prints_hi_to_the_project_author() -> None:
    expected = 'Hello, Your name (or your organization/company/team)!'
    actual = get_hello('Your name (or your organization/company/team)')
    assert actual == expected
