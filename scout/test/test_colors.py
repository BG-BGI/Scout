import pytest

from scout.core.colors import parse_hex_color as p


def test_parse_hex_color_forms():
    assert p('#FF8000') == (255, 128, 0)
    assert p('ff8000') == (255, 128, 0)
    assert p('0xFF8000') == (255, 128, 0)
    assert p('#abc') == (0xAA, 0xBB, 0xCC)
    assert p('') == (0, 0, 0)
    assert p(None) == (0, 0, 0)


def test_parse_hex_color_rejects_garbage():
    for bad in ('#12', 'gggggg', '#12345', 'xyz'):
        with pytest.raises(ValueError):
            p(bad)
