"""載入頁面樣式與腳本，並讓多個輸出頁面共用同一份離線 Plotly runtime。"""

from __future__ import annotations

import os
from functools import cache, lru_cache
from importlib.resources import files
from pathlib import Path

from plotly.offline import get_plotlyjs

ASSET_DIRECTORY = "assets"
ASSET_FILENAME = "plotly.min.js"


@cache
def read_asset(name: str) -> str:
    """讀取 static/ 內的 CSS 或 JS。

    樣式與腳本放在真正的 .css / .js 檔而不是 Python 字串，才能被編輯器與
    linter 處理，也不必為了 f-string 把每個大括號寫成兩個。
    """

    return files(__package__).joinpath("static", name).read_text(encoding="utf-8")


@lru_cache(maxsize=1)
def _runtime() -> str:
    return get_plotlyjs()


def write_plotly_asset(page_path: str | Path, asset_root: str | Path | None = None) -> str:
    """寫出共用 runtime，並回傳頁面可直接使用的相對 src。

    每頁內嵌一份 runtime 會讓輸出目錄膨脹到數十 MB；改成共用檔後仍然只依賴
    本機檔案，file:// 開啟不需要網路或 CDN。asset_root 未指定時放在頁面同層。
    """

    page = Path(page_path)
    root = Path(asset_root) if asset_root is not None else page.parent
    asset = root / ASSET_DIRECTORY / ASSET_FILENAME
    payload = _runtime()
    # 版本升級後檔案大小會改變，因此不能只用「檔案已存在」當作最新。
    if not asset.exists() or asset.stat().st_size != len(payload.encode("utf-8")):
        asset.parent.mkdir(parents=True, exist_ok=True)
        asset.write_text(payload, encoding="utf-8")
    return Path(os.path.relpath(asset, start=page.parent)).as_posix()
