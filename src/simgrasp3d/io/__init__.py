"""點雲、感測與運動時間序列的輸入輸出。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Protocol, TypeVar


class _FromDict(Protocol):
    @classmethod
    def from_dict(cls, data: dict) -> _FromDict: ...


SpecT = TypeVar("SpecT", bound=_FromDict)


def load_spec(path: str | Path, spec_type: type[SpecT]) -> SpecT:
    """讀取 UTF-8 JSON 設定並交給對應的 dataclass 驗證。"""

    with Path(path).open("r", encoding="utf-8") as stream:
        return spec_type.from_dict(json.load(stream))
