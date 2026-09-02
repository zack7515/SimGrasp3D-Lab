"""集中管理 3D 學習介面的色彩、字體與 Plotly 版面語言。"""

from __future__ import annotations

from typing import Any


VACUUM = "#11181D"
TITANIUM = "#2A3942"
SLATE = "#60727A"
CERAMIC = "#F4F7F6"
SCANLINE = "#D6E0DE"
LASER = "#0AA58F"
LASER_DARK = "#087568"
AMBER = "#E7A33B"
FAULT = "#C54437"
VIOLET = "#7656A5"
BLUE = "#2F7896"

DISPLAY_FONT = (
    '"Bahnschrift SemiCondensed", "DIN Alternate", "Arial Narrow", sans-serif'
)
BODY_FONT = (
    'Aptos, "Noto Sans TC", "PingFang TC", "Microsoft JhengHei", sans-serif'
)
MONO_FONT = (
    '"JetBrains Mono", "IBM Plex Mono", "SFMono-Regular", Consolas, monospace'
)


def instrument_layout(
    *,
    height: int,
    margin: dict[str, int],
    title: str | dict[str, Any] | None = None,
) -> dict[str, Any]:
    """回傳跨視圖一致的量測平台版面設定。"""

    return {
        "template": "plotly_white",
        "height": height,
        "margin": margin,
        "title": title,
        "paper_bgcolor": CERAMIC,
        "plot_bgcolor": CERAMIC,
        "font": {"family": BODY_FONT, "color": VACUUM, "size": 12},
        "hoverlabel": {
            "bgcolor": VACUUM,
            "bordercolor": LASER,
            "font": {"family": MONO_FONT, "color": "#FFFFFF", "size": 11},
        },
    }


def scene_axes(title: str, axis_range: list[float] | None = None) -> dict[str, Any]:
    """建立低干擾但仍可讀取尺度的 3D 座標軸。"""

    axis: dict[str, Any] = {
        "title": title,
        "showspikes": False,
        "showbackground": True,
        "backgroundcolor": "#EDF2F0",
        "gridcolor": "#CAD7D4",
        "linecolor": "#91A29F",
        "zerolinecolor": "#91A29F",
        "tickfont": {"family": MONO_FONT, "size": 10, "color": SLATE},
    }
    if axis_range is not None:
        axis["range"] = axis_range
    return axis
