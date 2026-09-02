"""匯出醫院案例摘要、指標與可重播時間序列。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from simgrasp3d.models.hospital import HospitalCaseResult, HospitalSuiteResult


def _case_summary(case: HospitalCaseResult) -> dict[str, object]:
    """把案例結果轉為不包含大型陣列的可稽核摘要。"""

    return {
        "case_id": case.spec.case_id,
        "order": case.spec.order,
        "title": case.spec.title,
        "domain": case.spec.domain,
        "risk_level": case.spec.risk_level,
        "maturity": case.spec.maturity,
        "engine": case.engine,
        "safety_scope": case.safety_scope,
        "summary": case.summary,
        "frame_rate_hz": case.frame_rate_hz,
        "frame_count": len(case.time_s),
        "duration_s": float(case.time_s[-1]),
        "assumptions": list(case.assumptions),
        "metrics": [
            {
                "key": metric.key,
                "label": metric.label,
                "value": metric.value,
                "unit": metric.unit,
                "direction": metric.direction,
                "limit": metric.limit,
                "passed": metric.passed,
                "calibrated": metric.calibrated,
            }
            for metric in case.metrics
        ],
        "events": [
            {
                "time_s": event.time_s,
                "phase": event.phase,
                "message": event.message,
                "severity": event.severity,
            }
            for event in case.events
        ],
    }


def export_hospital_suite(
    output_dir: str | Path,
    result: HospitalSuiteResult,
) -> dict[str, Path]:
    """輸出 suite JSON、各案例 JSON 與壓縮 NPZ。"""

    destination = Path(output_dir)
    data_dir = destination / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    paths: dict[str, Path] = {}
    summaries = [_case_summary(case) for case in result.cases]
    suite_path = data_dir / "suite_summary.json"
    suite_path.write_text(
        json.dumps(
            {
                "name": result.spec.name,
                "seed": result.spec.seed,
                "frame_rate_hz": result.spec.frame_rate_hz,
                "simulation_only": True,
                "cases": summaries,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    paths["summary"] = suite_path

    for case, summary in zip(result.cases, summaries, strict=True):
        case_json = data_dir / f"{case.spec.case_id}.json"
        case_json.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        arrays: dict[str, np.ndarray] = {
            "time_s": case.time_s,
            "phases": np.asarray(case.phases, dtype=np.str_),
        }
        for track in case.tracks:
            safe_name = track.name.replace(" ", "_")
            arrays[f"track_{safe_name}_world"] = track.world_positions
            arrays[f"track_{safe_name}_observed"] = track.observed_positions
        for signal_name, values in case.signals.items():
            arrays[f"signal_{signal_name.replace(' ', '_')}"] = np.asarray(values)
        case_npz = data_dir / f"{case.spec.case_id}.npz"
        np.savez_compressed(case_npz, **arrays)
        paths[f"{case.spec.case_id}_json"] = case_json
        paths[f"{case.spec.case_id}_npz"] = case_npz
    return paths

