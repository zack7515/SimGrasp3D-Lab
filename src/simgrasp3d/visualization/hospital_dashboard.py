"""輸出醫院案例索引與各案例的雙視窗動畫分析頁。"""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Any

import numpy as np
import plotly.graph_objects as go
from plotly.io import to_html
from plotly.offline import get_plotlyjs
from plotly.subplots import make_subplots

from simgrasp3d.models.hospital import (
    HospitalAsset,
    HospitalCaseResult,
    HospitalMetric,
    HospitalSuiteResult,
    HospitalTrack,
)
from simgrasp3d.visualization.theme import MONO_FONT


_CSS = """
:root{--ink:#17242b;--graphite:#23323a;--steel:#647780;--paper:#f6f8f7;--sterile:#e5eeec;--line:#cad7d4;--blue:#227c9d;--teal:#0a9b83;--teal-soft:#d8eee9;--coral:#d95d50;--coral-soft:#f8dfdc;--iodine:#d89a35;--iodine-soft:#f5e9d3;--display:"Bahnschrift SemiCondensed","DIN Alternate","Arial Narrow",sans-serif;--body:Aptos,"Noto Sans TC","PingFang TC","Microsoft JhengHei",sans-serif;--mono:"JetBrains Mono","IBM Plex Mono",Consolas,monospace}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;color:var(--ink);background:var(--sterile);font-family:var(--body);line-height:1.5}a{color:inherit}a:focus-visible{outline:3px solid var(--iodine);outline-offset:3px}.skip{position:fixed;z-index:100;top:8px;left:8px;padding:8px 12px;background:#fff;transform:translateY(-180%)}.skip:focus{transform:none}
.topbar{position:sticky;z-index:30;top:0;display:flex;justify-content:space-between;align-items:center;gap:20px;min-height:48px;padding:0 24px;color:#eef8f6;background:rgba(23,36,43,.96);border-bottom:1px solid #3d515a;backdrop-filter:blur(12px)}.brand,.case-state,.eyebrow,.badge,.metric-key,.event-time,.case-code{font-family:var(--mono);text-transform:uppercase;letter-spacing:.09em}.brand{color:inherit;font-size:10px;text-decoration:none}.brand+.brand{margin-left:10px;color:#8ce0d3}.case-state{color:#9fb3ba;font-size:9px}
.layout{display:grid;grid-template-columns:216px minmax(0,1fr);min-height:calc(100vh - 48px)}.spine{position:sticky;top:48px;align-self:start;height:calc(100vh - 48px);overflow-y:auto;padding:18px 14px 28px;color:#eaf3f2;background:var(--graphite);border-right:1px solid #49606a}.spine-label{margin:0 8px 14px;color:#88a0aa;font-family:var(--mono);font-size:9px;letter-spacing:.12em}.spine-link{position:relative;display:grid;grid-template-columns:32px 1fr;gap:9px;align-items:center;min-height:54px;padding:7px 6px;border-top:1px solid #40535b;color:#aebdc2;text-decoration:none}.spine-link:last-child{border-bottom:1px solid #40535b}.spine-link:hover,.spine-link.current{color:#fff;background:#2b3e47}.spine-link.current:after{content:"";position:absolute;top:8px;bottom:8px;right:-14px;width:4px;background:var(--teal)}.spine-index{display:grid;width:27px;height:27px;place-items:center;color:#fff;border:1px solid #6f858e;font-family:var(--mono);font-size:9px}.current .spine-index{color:var(--ink);background:#8ce0d3;border-color:#8ce0d3}.spine-copy strong,.spine-copy span{display:block}.spine-copy strong{font-size:11px}.spine-copy span{margin-top:2px;color:#7f969f;font-family:var(--mono);font-size:8px}
.page{min-width:0}.hero{position:relative;overflow:hidden;display:grid;grid-template-columns:minmax(0,1fr) 270px;gap:36px;min-height:275px;padding:38px clamp(24px,4vw,62px) 34px;color:#f5fbfa;background:var(--ink)}.hero:after{content:"";position:absolute;width:430px;height:430px;right:-155px;top:-250px;border:68px solid rgba(10,155,131,.18);border-radius:50%}.eyebrow{margin:0 0 12px;color:#8ce0d3;font-size:10px}.hero h1{margin:0;max-width:900px;font-family:var(--display);font-size:clamp(37px,5.5vw,72px);line-height:.92;letter-spacing:-.025em}.hero-summary{max-width:760px;margin:20px 0 0;color:#b9c9cd;font-size:14px}.hero-meta{position:relative;z-index:1;align-self:end;border-top:1px solid #43565f}.meta-row{display:grid;grid-template-columns:86px 1fr;gap:10px;padding:9px 0;border-bottom:1px solid #43565f;font-size:11px}.meta-row span{color:#8299a2;font-family:var(--mono);font-size:8px;text-transform:uppercase;letter-spacing:.1em}
.safety{display:grid;grid-template-columns:180px 1fr;gap:18px;align-items:center;padding:13px clamp(24px,4vw,62px);color:#5b3b0c;background:var(--iodine-soft);border-bottom:1px solid #dfc48f}.safety strong{font-family:var(--mono);font-size:10px;letter-spacing:.08em}.safety span{font-size:12px}.content{padding:22px clamp(18px,3vw,44px) 44px}.section{margin-bottom:16px;border:1px solid var(--line);background:var(--paper);box-shadow:0 9px 25px rgba(23,36,43,.06)}.section-head{display:flex;justify-content:space-between;gap:18px;align-items:flex-end;padding:14px 16px;border-bottom:1px solid var(--line);background:#fff}.section-head h2{margin:0;font-family:var(--display);font-size:20px}.section-head p{margin:2px 0 0;color:var(--steel);font-size:11px}.badge{flex:0 0 auto;padding:6px 9px;color:#0b6c5d;background:var(--teal-soft);font-size:8px}.plot{min-height:640px}.plot>div,.signal-plot>div{width:100%!important}.view-key{display:grid;grid-template-columns:1fr 1fr;border-top:1px solid var(--line)}.view-key div{padding:10px 14px;color:var(--steel);font-size:10px}.view-key div+div{border-left:1px solid var(--line)}.view-key strong{color:var(--ink);font-family:var(--mono);font-size:9px}
.metric-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr))}.metric{position:relative;min-height:128px;padding:16px;border-right:1px solid var(--line);border-bottom:1px solid var(--line)}.metric:before{content:"";position:absolute;top:0;right:0;left:0;height:3px;background:var(--steel)}.metric.pass:before{background:var(--teal)}.metric.fail:before{background:var(--coral)}.metric-key{color:var(--steel);font-size:8px}.metric-value{display:block;margin:15px 0 8px;font-family:var(--display);font-size:32px;line-height:1}.metric-limit{color:var(--steel);font-size:10px}.metric-status{float:right;font-family:var(--mono);font-size:9px}.pass .metric-status{color:#087565}.fail .metric-status{color:var(--coral)}.signal-plot{min-height:280px}.detail-grid{display:grid;grid-template-columns:1.15fr .85fr}.detail-grid>div{min-width:0;padding:16px}.detail-grid>div+div{border-left:1px solid var(--line)}.detail-grid h3{margin:0 0 10px;font-family:var(--display);font-size:16px}.event-table{width:100%;border-collapse:collapse;font-size:11px}.event-table td{padding:8px 7px;border-bottom:1px solid #dce5e2;vertical-align:top}.event-time{width:72px;color:var(--blue);font-size:8px}.assumptions{margin:0;padding-left:18px;color:#52656d;font-size:11px}.assumptions li+li{margin-top:8px}.pager{display:flex;justify-content:space-between;gap:12px;margin-top:22px}.pager a{min-width:145px;padding:12px 14px;color:#fff;background:var(--graphite);font-family:var(--mono);font-size:9px;text-decoration:none}.pager a:last-child{text-align:right}.disabled{visibility:hidden}
@media(max-width:980px){.layout{grid-template-columns:1fr}.spine{position:static;display:flex;gap:4px;height:auto;padding:8px;overflow-x:auto}.spine-label{display:none}.spine-link{flex:0 0 122px;grid-template-columns:26px 1fr;border:1px solid #40535b}.spine-link.current:after{top:auto;right:7px;bottom:-8px;left:7px;width:auto;height:3px}.hero{grid-template-columns:1fr}.detail-grid{grid-template-columns:1fr}.detail-grid>div+div{border-left:0;border-top:1px solid var(--line)}}
@media(max-width:650px){.topbar{padding:0 12px}.case-state{display:none}.hero{min-height:0;padding:30px 20px}.safety{grid-template-columns:1fr;gap:4px;padding:12px 20px}.content{padding:12px 8px 32px}.badge{display:none}.plot{min-height:560px}.view-key{grid-template-columns:1fr}.view-key div+div{border-left:0;border-top:1px solid var(--line)}}
@media(prefers-reduced-motion:reduce){*{scroll-behavior:auto!important;animation:none!important;transition:none!important}}
"""


_INDEX_CSS = _CSS + """
.index-hero{display:grid;grid-template-columns:minmax(0,1.25fr) minmax(280px,.75fr);min-height:420px;color:#f4fbfa;background:var(--ink)}.index-copy{padding:clamp(38px,7vw,94px);align-self:end}.index-copy h1{margin:0;max-width:850px;font-family:var(--display);font-size:clamp(48px,8vw,108px);letter-spacing:-.035em;line-height:.83}.index-copy p:last-child{max-width:680px;margin:28px 0 0;color:#aebfc4;font-size:14px}.cover{position:relative;overflow:hidden;display:grid;align-content:center;padding:36px;background:#20343d;border-left:1px solid #47606a}.cover:before{content:"";position:absolute;width:390px;height:390px;top:-150px;right:-145px;border:64px solid rgba(10,155,131,.18);border-radius:50%}.cover-line{position:relative;display:grid;grid-template-columns:50px 1fr;gap:12px;padding:13px 0;border-top:1px solid #405761}.cover-line:last-child{border-bottom:1px solid #405761}.cover-line strong{font-family:var(--display);font-size:20px}.cover-line span{align-self:center;color:#86a0a9;font-family:var(--mono);font-size:9px}.notice{padding:14px clamp(24px,6vw,84px);color:#5c3d10;background:var(--iodine-soft);border-bottom:1px solid #dfc48f;font-size:12px}.index-main{width:min(1420px,100%);margin:0 auto;padding:34px clamp(16px,4vw,54px) 60px}.title-row{display:flex;justify-content:space-between;gap:24px;align-items:end;margin:0 0 16px}.title-row h2{margin:0;font-family:var(--display);font-size:28px}.title-row p{max-width:620px;margin:0;color:var(--steel);font-size:12px}.sequence{display:grid;grid-template-columns:repeat(7,minmax(0,1fr));margin-bottom:32px;border:1px solid var(--line);background:#fff}.step{position:relative;min-width:0;padding:18px 12px;border-right:1px solid var(--line);text-decoration:none}.step:last-child{border-right:0}.step:before{content:"";position:absolute;top:0;right:0;left:0;height:4px;background:var(--teal);transform:scaleX(0);transform-origin:left;transition:transform .2s}.step:hover:before{transform:scaleX(1)}.case-code{color:var(--blue);font-size:9px}.step strong{display:block;min-height:42px;margin-top:10px;font-size:12px}.level{color:var(--steel);font-family:var(--mono);font-size:8px}.cases{display:grid;gap:12px}.case{display:grid;grid-template-columns:90px minmax(0,1.2fr) minmax(200px,.8fr) 130px;align-items:center;min-height:118px;color:inherit;background:var(--paper);border:1px solid var(--line);text-decoration:none;transition:transform .18s,box-shadow .18s}.case:hover{transform:translateY(-2px);box-shadow:0 12px 28px rgba(23,36,43,.10)}.case-index{display:grid;height:100%;place-items:center;color:#fff;background:var(--graphite);font-family:var(--display);font-size:32px}.case-copy,.case-facts,.case-status{padding:18px}.case-copy h3{margin:0;font-family:var(--display);font-size:23px}.case-copy p{margin:7px 0 0;color:var(--steel);font-size:11px}.case-facts{display:grid;gap:7px;border-left:1px solid var(--line);font-size:10px}.fact{margin-right:8px;color:var(--steel);font-family:var(--mono);font-size:8px;text-transform:uppercase}.case-status{align-self:stretch;display:grid;align-content:center;justify-items:end;border-left:1px solid var(--line)}.status{padding:6px 8px;color:#087565;background:var(--teal-soft);font-family:var(--mono);font-size:8px}.status.fail{color:#9d3e35;background:var(--coral-soft)}.risk{margin-top:8px;color:var(--steel);font-family:var(--mono);font-size:8px;text-transform:uppercase}.matrix{width:100%;margin-top:34px;border-collapse:collapse;background:#fff;font-size:11px}.matrix th,.matrix td{padding:11px 12px;border:1px solid var(--line);text-align:left}.matrix th{color:var(--steel);font-family:var(--mono);font-size:8px;text-transform:uppercase}.back{display:inline-block;margin-top:22px;padding:11px 14px;color:#fff;background:var(--graphite);font-family:var(--mono);font-size:9px;text-decoration:none}
.home-jump{display:inline-block;margin-bottom:18px;color:#8ce0d3;font-family:var(--mono);font-size:9px;letter-spacing:.08em;text-decoration:none;text-transform:uppercase}
@media(max-width:950px){.index-hero{grid-template-columns:1fr}.cover{border-left:0;border-top:1px solid #47606a}.sequence{grid-template-columns:repeat(4,1fr)}.case{grid-template-columns:64px 1fr 120px}.case-facts{display:none}}@media(max-width:620px){.index-copy{padding:42px 24px}.sequence{display:flex;overflow-x:auto}.step{flex:0 0 125px}.case{grid-template-columns:54px 1fr}.case-status{display:none}.matrix{display:block;overflow-x:auto;white-space:nowrap}.title-row{display:block}.title-row p{margin-top:8px}}
"""


def _asset_trace(asset: HospitalAsset, scene_name: str) -> Any:
    if asset.kind == "polyline":
        assert asset.points is not None
        return go.Scatter3d(
            x=asset.points[:, 0], y=asset.points[:, 1], z=asset.points[:, 2],
            mode="lines", name=asset.name, line={"color": asset.color, "width": 8},
            opacity=asset.opacity, hovertemplate=f"<b>{escape(asset.name)}</b><extra></extra>",
            scene=scene_name, showlegend=False,
        )
    assert asset.center is not None and asset.size is not None
    center, half = np.asarray(asset.center), np.asarray(asset.size) / 2.0
    vertices = np.asarray([center + half * signs for signs in (
        (-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)
    )])
    return go.Mesh3d(
        x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
        i=[0,0,4,4,0,0,1,1,2,2,3,3], j=[1,2,5,6,1,5,2,6,3,7,0,4], k=[2,3,6,7,5,4,6,5,7,6,4,7],
        name=asset.name, color=asset.color, opacity=asset.opacity, flatshading=True,
        hovertemplate=f"<b>{escape(asset.name)}</b><extra></extra>", scene=scene_name, showlegend=False,
    )


def _track_trace(track: HospitalTrack, index: int, observed: bool, scene_name: str) -> go.Scatter3d:
    points = track.observed_positions[index] if observed else track.world_positions[index]
    suffix = " / OBS" if observed else " / GT"
    return go.Scatter3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2], mode=track.style,
        name=f"{track.name}{suffix}", line={"color": track.color, "width": track.width},
        marker={"color": track.color, "size": track.marker_size, "line": {"color": "#fff", "width": 1}},
        hovertemplate=f"<b>{escape(track.name)}</b><br>x=%{{x:.3f}} m<br>y=%{{y:.3f}} m<br>z=%{{z:.3f}} m<extra></extra>",
        scene=scene_name,
    )


def _bounds(case: HospitalCaseResult) -> tuple[np.ndarray, np.ndarray]:
    point_sets: list[np.ndarray] = []
    for asset in case.assets:
        if asset.points is not None:
            point_sets.append(asset.points)
        elif asset.center is not None and asset.size is not None:
            center, half = np.asarray(asset.center), np.asarray(asset.size) / 2.0
            point_sets.extend((center - half, center + half))
    for track in case.tracks:
        point_sets.extend((track.world_positions.reshape(-1, 3), track.observed_positions.reshape(-1, 3)))
    values = np.vstack(point_sets)
    minimum, maximum = np.nanmin(values, axis=0), np.nanmax(values, axis=0)
    padding = np.maximum((maximum - minimum) * 0.10, 0.08)
    return minimum - padding, maximum + padding


def build_hospital_case_figure(case: HospitalCaseResult) -> go.Figure:
    """建立真值世界與含誤差安全模型的同步雙 3D 動畫。"""

    figure = make_subplots(
        rows=1, cols=2, specs=[[{"type": "scene"}, {"type": "scene"}]],
        subplot_titles=("GROUND TRUTH / 原始世界", "OBSERVATION + SAFETY / 觀測與規則"),
        horizontal_spacing=0.025,
    )
    for asset in case.assets:
        if not asset.analysis_only:
            figure.add_trace(_asset_trace(asset, "scene"), row=1, col=1)
        figure.add_trace(_asset_trace(asset, "scene2"), row=1, col=2)
    dynamic_indexes: list[int] = []
    sources: list[tuple[HospitalTrack, bool, str]] = []
    for observed, column, scene_name in ((False, 1, "scene"), (True, 2, "scene2")):
        for track in case.tracks:
            dynamic_indexes.append(len(figure.data))
            sources.append((track, observed, scene_name))
            figure.add_trace(_track_trace(track, 0, observed, scene_name), row=1, col=column)
            if track.world_positions.shape[1] == 1:
                path = track.observed_positions[:, 0] if observed else track.world_positions[:, 0]
                figure.add_trace(go.Scatter3d(
                    x=path[:, 0], y=path[:, 1], z=path[:, 2], mode="lines",
                    name=f"{track.name} 完整路徑", line={"color": track.color, "width": 3, "dash": "dash"},
                    opacity=0.38, hoverinfo="skip", showlegend=False, scene=scene_name,
                ), row=1, col=column)
    figure.frames = tuple(go.Frame(
        name=f"{index:04d}", data=[_track_trace(*source[:1], index=index, observed=source[1], scene_name=source[2]) for source in sources], traces=dynamic_indexes,
    ) for index in range(len(case.time_s)))
    minimum, maximum = _bounds(case)
    def axis(label: str, low: float, high: float) -> dict[str, Any]:
        return {"title": label, "range": [float(low), float(high)], "gridcolor": "#cbd8d5", "backgroundcolor": "#eef3f1", "showbackground": True, "zerolinecolor": "#9badab"}
    scene = {
        "xaxis": axis("X（m）", minimum[0], maximum[0]), "yaxis": axis("Y（m）", minimum[1], maximum[1]), "zaxis": axis("Z（m）", minimum[2], maximum[2]),
        "aspectmode": "data", "bgcolor": "#f6f8f7", "camera": {"eye": {"x": 1.4, "y": -1.55, "z": 1.05}}, "dragmode": "orbit",
    }
    frame_ms = int(round(1000 / case.frame_rate_hz))
    slider_steps = [{
        "args": [[f"{index:04d}"], {"frame": {"duration": 0, "redraw": True}, "mode": "immediate", "transition": {"duration": 0}}],
        "label": f"{value:.1f}s" if index % max(1, case.frame_rate_hz) == 0 else "", "method": "animate",
    } for index, value in enumerate(case.time_s)]
    figure.update_layout(
        height=640, margin={"l": 0, "r": 0, "t": 48, "b": 100}, paper_bgcolor="#f6f8f7",
        font={"family": "Aptos, Noto Sans TC, sans-serif", "color": "#17242b", "size": 11},
        scene=scene, scene2=scene, legend={"orientation": "h", "x": 0, "y": 1.06, "font": {"size": 9}},
        sliders=[{"active": 0, "currentvalue": {"prefix": "TIME / ", "font": {"family": MONO_FONT, "size": 10}}, "pad": {"t": 42}, "steps": slider_steps}],
        updatemenus=[{"type": "buttons", "direction": "left", "x": 0, "y": -0.10, "showactive": False, "buttons": [
            {"label": "▶ 播放", "method": "animate", "args": [None, {"frame": {"duration": frame_ms, "redraw": True}, "fromcurrent": True, "transition": {"duration": 0}}]},
            {"label": "Ⅱ 暫停", "method": "animate", "args": [[None], {"frame": {"duration": 0, "redraw": False}, "mode": "immediate"}]},
            {"label": "↺ 從頭", "method": "animate", "args": [[frame.name for frame in figure.frames], {"frame": {"duration": frame_ms, "redraw": True}, "mode": "immediate"}]},
        ]}],
    )
    return figure


def build_hospital_signal_figure(case: HospitalCaseResult) -> go.Figure:
    """建立案例時間訊號的共用時間軸圖。"""

    names = list(case.signals)
    figure = make_subplots(rows=len(names), cols=1, shared_xaxes=True, vertical_spacing=0.10)
    colors = ("#227c9d", "#d89a35", "#d95d50", "#745b9e")
    for row, name in enumerate(names, start=1):
        figure.add_trace(go.Scatter(
            x=case.time_s, y=case.signals[name], mode="lines", name=name,
            line={"color": colors[(row - 1) % len(colors)], "width": 2.5},
            hovertemplate=f"<b>{escape(name)}</b><br>t=%{{x:.2f}} s<br>value=%{{y:.4f}} {escape(case.signal_units[name])}<extra></extra>",
        ), row=row, col=1)
        figure.update_yaxes(title_text=case.signal_units[name], row=row, col=1, gridcolor="#d8e2df", title_font={"size": 9})
    figure.update_xaxes(title_text="TIME（s）", row=len(names), col=1, gridcolor="#d8e2df")
    figure.update_layout(
        height=max(280, 135 * len(names)), margin={"l": 55, "r": 22, "t": 20, "b": 45},
        paper_bgcolor="#f6f8f7", plot_bgcolor="#f6f8f7", showlegend=True,
        font={"family": "Aptos, Noto Sans TC, sans-serif", "color": "#17242b", "size": 10},
        legend={"orientation": "h", "x": 0, "y": 1.06}, hovermode="x unified",
    )
    return figure


def _metric_display(metric: HospitalMetric) -> str:
    if metric.unit == "m":
        return f"{metric.value * 1000:.2f} mm"
    if metric.unit == "ratio":
        return f"{metric.value * 100:.1f}%"
    if metric.unit == "count":
        return str(int(round(metric.value)))
    if metric.unit == "deg":
        return f"{metric.value:.2f}°"
    return f"{metric.value:.2f} {metric.unit}".strip()


def _limit_display(metric: HospitalMetric) -> str:
    if metric.limit is None:
        return "資訊指標／未設安全門檻"
    symbol = {"maximum": "≤", "minimum": "≥", "exact": "="}.get(metric.direction, "")
    limit_metric = HospitalMetric(metric.key, metric.label, metric.limit, metric.unit)
    return f"教學門檻 {symbol} {_metric_display(limit_metric)}"


def _risk_label(risk: str) -> str:
    return {"low": "低", "medium": "中", "high": "高", "very_high": "極高"}[risk]


def _case_filename(case: HospitalCaseResult) -> str:
    return f"case-{case.spec.order:02d}-{case.spec.case_id}.html"


def _spine(suite: HospitalSuiteResult, current: HospitalCaseResult) -> str:
    links: list[str] = []
    for case in suite.cases:
        is_current = case.spec.case_id == current.spec.case_id
        class_name = "spine-link current" if is_current else "spine-link"
        current_attr = ' aria-current="page"' if is_current else ""
        links.append(
            f'<a class="{class_name}" href="{_case_filename(case)}"{current_attr}>'
            f'<span class="spine-index">H{case.spec.order}</span><span class="spine-copy">'
            f'<strong>{escape(case.spec.short_title)}</strong><span>{escape(case.spec.risk_level)}</span></span></a>'
        )
    return "".join(links)


def _write_case_page(
    suite: HospitalSuiteResult,
    case: HospitalCaseResult,
    destination: Path,
) -> None:
    """輸出一個可離線開啟的案例分析頁。"""

    animation = to_html(
        build_hospital_case_figure(case), include_plotlyjs=False, full_html=False,
        div_id=f"hospital-{case.spec.case_id}",
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    signals = to_html(
        build_hospital_signal_figure(case), include_plotlyjs=False, full_html=False,
        div_id=f"signals-{case.spec.case_id}",
        config={"displaylogo": False, "responsive": True},
    )
    metric_cards: list[str] = []
    for metric in case.metrics:
        state = "pass" if metric.passed is True else "fail" if metric.passed is False else "info"
        status = "PASS" if metric.passed is True else "STOP" if metric.passed is False else "INFO"
        metric_cards.append(
            f'<div class="metric {state}"><span class="metric-key">{escape(metric.key)}</span>'
            f'<span class="metric-status">{status}</span><strong class="metric-value">{escape(_metric_display(metric))}</strong>'
            f'<div>{escape(metric.label)}</div><div class="metric-limit">{escape(_limit_display(metric))}｜未校正</div></div>'
        )
    event_rows = "".join(
        f'<tr><td class="event-time">{event.time_s:05.2f} s</td><td>{escape(event.phase)}</td><td>{escape(event.message)}</td></tr>'
        for event in case.events
    )
    assumptions = "".join(f"<li>{escape(item)}</li>" for item in case.assumptions)
    index = suite.cases.index(case)
    previous_link = (
        '<span class="disabled"></span>'
        if index == 0
        else f'<a href="{_case_filename(suite.cases[index - 1])}">← H{suite.cases[index - 1].spec.order} {escape(suite.cases[index - 1].spec.short_title)}</a>'
    )
    next_link = (
        '<span class="disabled"></span>'
        if index == len(suite.cases) - 1
        else f'<a href="{_case_filename(suite.cases[index + 1])}">H{suite.cases[index + 1].spec.order} {escape(suite.cases[index + 1].spec.short_title)} →</a>'
    )
    document = f"""<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H{case.spec.order} {escape(case.spec.title)}｜SimGrasp3D Hospital</title><style>{_CSS}</style><script>{get_plotlyjs()}</script></head>
<body><a class="skip" href="#case-main">跳到案例內容</a>
<header class="topbar"><div><a class="brand" href="../index.html">← SimGrasp3D main</a><a class="brand" href="index.html">Hospital learning suite</a></div><span class="case-state">SIMULATION ONLY · NOT FOR CLINICAL USE</span></header>
<div class="layout"><aside class="spine" aria-label="案例病歷索引"><p class="spine-label">CASE CHART / 依序學習</p>{_spine(suite, case)}</aside>
<div class="page"><main id="case-main"><section class="hero"><div><p class="eyebrow">Hospital case H{case.spec.order:02d} / {escape(case.spec.domain)}</p>
<h1>{escape(case.spec.title)}</h1><p class="hero-summary">{escape(case.summary)}</p></div>
<div class="hero-meta"><div class="meta-row"><span>風險</span><strong>{_risk_label(case.spec.risk_level)}／{escape(case.spec.risk_level)}</strong></div>
<div class="meta-row"><span>成熟度</span><strong>{escape(case.spec.maturity)}</strong></div><div class="meta-row"><span>引擎</span><strong>{escape(case.engine)}</strong></div>
<div class="meta-row"><span>資料</span><strong>{len(case.time_s)} frames · {case.time_s[-1]:.1f} s</strong></div></div></section>
<div class="safety"><strong>SAFETY BOUNDARY</strong><span>{escape(case.safety_scope)}</span></div>
<div class="content">
<section class="section"><header class="section-head"><div><h2>同步情境重播</h2><p>拖曳、縮放或播放逐幀比較真值與觀測；兩邊共用同一時間軸。</p></div><span class="badge">GT ↔ OBSERVATION</span></header>
<div class="plot">{animation}</div><div class="view-key"><div><strong>GROUND TRUTH / 原始世界</strong> 完整幾何與名義軌跡</div><div><strong>OBSERVATION + SAFETY</strong> 含設定誤差的觀測、語意區及規則圖層</div></div></section>
<section class="section"><header class="section-head"><div><h2>教學安全閘門</h2><p>PASS 只對應本案例設定門檻；所有門檻皆未經醫療材料或臨床校正。</p></div><span class="badge">FAIL-CLOSED REVIEW</span></header><div class="metric-grid">{''.join(metric_cards)}</div></section>
<section class="section"><header class="section-head"><div><h2>連續訊號</h2><p>訊號由模擬時間序列計算，不是醫療監測值。</p></div><span class="badge">TIME SERIES</span></header><div class="signal-plot">{signals}</div></section>
<section class="section"><header class="section-head"><div><h2>事件與模型界線</h2><p>先看事件，再檢查哪些現實條件尚未進入模型。</p></div></header>
<div class="detail-grid"><div><h3>事件時間線</h3><table class="event-table"><tbody>{event_rows}</tbody></table></div><div><h3>必要假設</h3><ul class="assumptions">{assumptions}</ul></div></div></section>
<nav class="pager" aria-label="案例前後導覽">{previous_link}{next_link}</nav></div></main></div></div>
<script>window.addEventListener("load",function(){{setTimeout(function(){{Plotly.Plots.resize(document.getElementById("hospital-{case.spec.case_id}"));Plotly.Plots.resize(document.getElementById("signals-{case.spec.case_id}"));}},80);}});</script>
</body></html>"""
    destination.write_text(document, encoding="utf-8")


def _write_index(suite: HospitalSuiteResult, destination: Path) -> None:
    """輸出七個案例的序列、風險與成熟度索引。"""

    sequence = "".join(
        f'<a class="step" href="{_case_filename(case)}"><span class="case-code">H{case.spec.order:02d}</span>'
        f'<strong>{escape(case.spec.short_title)}</strong><span class="level">{escape(case.spec.maturity.split()[0])}</span></a>'
        for case in suite.cases
    )
    cards: list[str] = []
    matrix_rows: list[str] = []
    for case in suite.cases:
        stopped = any(metric.passed is False for metric in case.metrics)
        status_class = "status fail" if stopped else "status"
        status = "STOP" if stopped else "BASELINE PASS"
        cards.append(
            f'<a class="case" href="{_case_filename(case)}"><span class="case-index">H{case.spec.order}</span>'
            f'<div class="case-copy"><h3>{escape(case.spec.title)}</h3><p>{escape(case.summary)}</p></div>'
            f'<div class="case-facts"><div><span class="fact">Domain</span>{escape(case.spec.domain)}</div>'
            f'<div><span class="fact">Model</span>{escape(case.engine)}</div><div><span class="fact">Scope</span>{escape(case.safety_scope)}</div></div>'
            f'<div class="case-status"><span class="{status_class}">{status}</span><span class="risk">risk / {escape(case.spec.risk_level)}</span></div></a>'
        )
        matrix_rows.append(
            f'<tr><td>H{case.spec.order}</td><td>{escape(case.spec.title)}</td><td>{escape(case.spec.domain)}</td>'
            f'<td>{escape(case.spec.maturity)}</td><td>{_risk_label(case.spec.risk_level)}</td><td>{len(case.time_s)}</td><td>{case.time_s[-1]:.1f} s</td></tr>'
        )
    document = f"""<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>醫院機器人模擬學習套件｜SimGrasp3D</title><style>{_INDEX_CSS}</style></head><body>
<header class="index-hero"><div class="index-copy"><a class="home-jump" href="../index.html">← SimGrasp3D project home</a><p class="eyebrow">SimGrasp3D / Biomedical simulation track</p><h1>醫院不是一個場景，<br>而是七層風險。</h1>
<p>由封閉試管與器械盤開始，逐步進入柔性管路、院內物流、消毒覆蓋、假體接觸與導管研究。每一頁都把真值、觀測、規則、時間訊號與模型缺口放在一起。</p></div>
<div class="cover" aria-label="套件摘要"><div class="cover-line"><span>CASES</span><strong>{len(suite.cases)} 個可重播案例</strong></div>
<div class="cover-line"><span>SEED</span><strong>{suite.spec.seed}</strong></div><div class="cover-line"><span>RATE</span><strong>{suite.spec.frame_rate_hz} Hz</strong></div>
<div class="cover-line"><span>SCOPE</span><strong>TRAINING · SIMULATION ONLY</strong></div></div></header>
<div class="notice"><strong>研究界線：</strong> 本套件只用於軟體學習與假體／無病患情境，不提供診斷、治療、臨床控制或醫療器材符合性證據。</div>
<main class="index-main"><section><div class="title-row"><h2>依序學習</h2><p>順序同時代表新增的模型複雜度。H06、H07 是低保真研究預覽，不能因畫面可播放就解讀為物理可信。</p></div>
<nav class="sequence" aria-label="醫院案例順序">{sequence}</nav></section>
<section><div class="title-row"><h2>案例病歷索引</h2><p>每個案例有獨立頁面，避免物流、柔性物與生醫接觸共享錯誤的門檻或結論。</p></div><div class="cases">{''.join(cards)}</div></section>
<table class="matrix"><thead><tr><th>順序</th><th>案例</th><th>領域</th><th>成熟度</th><th>風險</th><th>幀數</th><th>時間</th></tr></thead><tbody>{''.join(matrix_rows)}</tbody></table>
<a class="back" href="../index.html">← 回到專案主頁</a> <a class="back" href="../simulation_report.html">查看桌面抓取 Stage 1–7 報告 →</a></main></body></html>"""
    destination.write_text(document, encoding="utf-8")


def write_hospital_dashboard(
    output_dir: str | Path,
    suite: HospitalSuiteResult,
) -> dict[str, Path]:
    """輸出多頁醫院模擬介面並回傳所有頁面路徑。"""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    pages: dict[str, Path] = {}
    index_path = destination / "index.html"
    _write_index(suite, index_path)
    pages["index"] = index_path
    for case in suite.cases:
        path = destination / _case_filename(case)
        _write_case_page(suite, case, path)
        pages[case.spec.case_id] = path
    return pages
