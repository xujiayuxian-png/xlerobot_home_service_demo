"""One local, dictionary-based text normalization for voice task boundaries."""

from opencc import OpenCC


_CONVERTER = OpenCC('t2s')


def simplified_text(text: str) -> str:
    return _CONVERTER.convert(text)
