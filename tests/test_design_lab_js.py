"""工作台 JS 與 Python 幾何實作的對拍測試。

`static/design_lab.js` 為了即時調參，在瀏覽器端重寫了一份線段距離運算。兩份
實作分開演化就會讓頁面上的路徑淨空閘門與 Python 管線給出不同答案，因此在這裡
用相同輸入比對兩者。
"""

import json
import re
import shutil
import subprocess
from importlib.resources import files

import numpy as np
import pytest

from simgrasp3d.geometry.collision import segment_distance

_JS_SOURCE = files("simgrasp3d.visualization").joinpath("static/design_lab.js").read_text(
    encoding="utf-8"
)
_HELPERS = ("add", "sub", "mul", "dot", "norm", "segmentDistance")


def _extract_js_geometry() -> str:
    """取出 JS 中的向量輔助函式與 segmentDistance。"""

    helper_line = re.search(r"^\s*const add=.*$", _JS_SOURCE, re.M)
    norm_line = re.search(r"^\s*const dot=.*$", _JS_SOURCE, re.M)
    body = re.search(
        r"^  function segmentDistance\(.*?^  \}$", _JS_SOURCE, re.M | re.S
    )
    assert helper_line and norm_line and body, "design_lab.js 的幾何區塊結構已改變"
    return "\n".join([helper_line.group(0), norm_line.group(0), body.group(0)])


@pytest.mark.skipif(shutil.which("node") is None, reason="需要 node 才能執行 JS 對拍")
def test_js_segment_distance_matches_python() -> None:
    generator = np.random.default_rng(7515)
    cases = generator.uniform(-1.0, 1.0, size=(200, 4, 3))
    # 退化線段（點對點、共線）是兩份實作最容易分歧的地方，因此另外補上。
    cases[0, 1] = cases[0, 0]
    cases[1, 3] = cases[1, 2]
    cases[2, 1] = cases[2, 0]
    cases[2, 3] = cases[2, 2]

    script = (
        _extract_js_geometry()
        + "\nconst cases = JSON.parse(process.argv[1]);\n"
        + "console.log(JSON.stringify(cases.map(c => segmentDistance(c[0],c[1],c[2],c[3]))));\n"
    )
    completed = subprocess.run(
        ["node", "-e", script, json.dumps(cases.tolist())],
        capture_output=True,
        text=True,
        check=True,
    )
    from_js = np.asarray(json.loads(completed.stdout))
    from_python = np.asarray(
        [segment_distance(case[0], case[1], case[2], case[3]) for case in cases]
    )

    assert np.allclose(from_js, from_python, atol=1e-9), (
        "design_lab.js 與 geometry/collision.py 的線段距離已經分歧"
    )


def test_js_geometry_helpers_are_still_present() -> None:
    """抽取用的名稱一旦改名，對拍測試會靜默失效，因此單獨守住。"""

    for name in _HELPERS:
        assert name in _JS_SOURCE, f"design_lab.js 缺少 {name}"
