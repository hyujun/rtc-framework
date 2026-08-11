"""#392 재발 방지 게이트 — 실제 robot_descriptions 모델 쌍에 센서를 발사한다.

`test_compare_mjcf_urdf.py` 는 합성 픽스처로 **툴의 로직**을 검증한다. 그건
툴이 정직한지만 보장할 뿐, 그 정직해진 툴이 *무엇을 겨냥하는지* 는 보장하지
않는다 — #392 (a) 가 발견한 것이 정확히 그 공백이었다 (센서를 자동 실행하는
곳이 저장소에 0곳). 이 파일이 그 공백을 메운다.

**로컬에서는 skip 된다.** colcon 이 pytest 를 `/usr/bin/python3` 로 돌리는데
(colcon 자체의 shebang) 거기엔 mujoco 가 없고, 이 저장소는 mujoco 를 `.venv`
에 둔다. CI 는 `pip install ... mujoco` 를 하므로 실제로 실행된다. skip 은
조용한 통과가 아니라 pytest 리포트에 남는다. 로컬에서 직접 돌리려면::

    ( cd <rtc_ws> && source <repo>/repo_scripts/scripts/setup_env.sh \
      && cd <repo> \
      && PYTHONPATH=rtc_tools:$PYTHONPATH \
         <rtc_ws>/.venv/bin/python -m pytest rtc_tools/test/test_real_model_pairs.py )

``PYTHONPATH`` 는 **덮어쓰지 말고 이어붙인다** — xacro 쌍이 ``import xacro`` 를
하는데 그 모듈은 ROS 가 ``PYTHONPATH`` 로만 노출하므로, ``PYTHONPATH=rtc_tools``
로 대입하면 ``ModuleNotFoundError: xacro`` 가 난다 (#414).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from rtc_tools.validation.compare_mjcf_urdf import (
    _detect_joints,
    _detect_link_mapping,
    _load_link_map,
    _resolve_alignment,
    compare,
)


def _robots_dir() -> Path | None:
    """ament share 우선, 없으면 in-source 레이아웃."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("robot_descriptions")) / "robots"
        if (share / "model_pairs.yaml").is_file():
            return share
    except Exception:
        pass
    in_source = Path(__file__).resolve().parents[2] / "robot_descriptions" / "robots"
    if (in_source / "model_pairs.yaml").is_file():
        return in_source
    return None


ROBOTS = _robots_dir()


def _pairs():
    if ROBOTS is None:
        return []
    data = yaml.safe_load((ROBOTS / "model_pairs.yaml").read_text())
    return data.get("pairs", [])


PAIRS = _pairs()


def test_manifest_is_present_and_non_empty():
    """게이트가 조용히 0개 쌍을 돌면 그건 통과가 아니라 미실행이다."""
    assert ROBOTS is not None, "robot_descriptions/robots/model_pairs.yaml 을 찾지 못했다"
    assert PAIRS, "model_pairs.yaml 에 쌍이 하나도 없다"


def _resolve_urdf(path: Path, tmp_path: Path) -> Path:
    """`*.urdf.xacro` 를 확장한다. 평범한 `.urdf` 는 그대로 통과.

    확장 실패는 **skip 이 아니라 테스트 실패**다. URDF 를 만들지 못한 쌍은
    검사하지 않은 쌍이고, 그것이 통과처럼 보이는 것이 #392 의 주제다.
    xacro import 도 여기서 한다 — 모듈 최상단에 두면 xacro 부재가 collection
    error 가 되어 이 파일의 다른 테스트까지 함께 죽는다.
    """
    if path.suffix != ".xacro":
        return path

    import xacro

    # `$(find robot_descriptions)` 를 쓰므로 ament index 에 그 패키지가 있어야
    # 한다 — in-source 경로만으로는 안 된다 (.github/ci-packages.yml 참조).
    doc = xacro.process_file(str(path))
    out = tmp_path / f"{path.stem.removesuffix('.urdf')}.urdf"
    out.write_text(doc.toprettyxml(indent="  "))
    return out


@pytest.mark.parametrize("pair", PAIRS, ids=[p["name"] for p in PAIRS])
def test_real_pair_has_no_mismatches(pair, capsys, tmp_path):
    # 구조 비교는 컴파일된 모델이 필요하다. 없으면 body count / total mass /
    # missing links 가 통째로 빠진 채 "Mismatches: 0" 이 찍힌다 (#385, #392).
    pytest.importorskip("mujoco")

    mjcf = ROBOTS / pair["mjcf"]
    urdf_src = ROBOTS / pair["urdf"]
    assert mjcf.is_file(), f"MJCF 없음: {mjcf}"
    assert urdf_src.is_file(), f"URDF 없음: {urdf_src}"
    urdf = _resolve_urdf(urdf_src, tmp_path)

    if pair.get("link_map"):
        link_map, fuse = _load_link_map(ROBOTS / pair["link_map"])
    else:
        link_map, fuse = _detect_link_mapping(mjcf, urdf), {}
    align = (
        _resolve_alignment(mjcf, urdf, *pair["align_frames"]) if pair.get("align_frames") else None
    )

    mismatches = compare(
        mjcf,
        urdf,
        link_map=link_map,
        joint_names=_detect_joints(mjcf, urdf),
        robot_label=pair["name"],
        align=align,
        tip_frames=tuple(pair["tip_frames"]) if pair.get("tip_frames") else None,
        fail_on_unverified=True,
        fuse=fuse,
    )
    out = capsys.readouterr().out
    # 실패 시 비교 리포트를 그대로 보여준다 — 어느 파라미터인지 알아야 고친다.
    assert mismatches == 0, f"{pair['name']}: {mismatches} mismatch(es)\n{out}"

    # warning 도 고정한다. 정당한 차이라 0 은 아니지만, 늘어나면 새로 생긴
    # 것이므로 근거와 함께 model_pairs.yaml 을 갱신해야 한다.
    expected = pair.get("expect_warnings", 0)
    actual = int(out.split("Warnings:")[-1].split("\n")[0].strip())
    assert actual == expected, (
        f"{pair['name']}: warnings {actual} != expected {expected} — "
        f"새 warning 이면 근거를 확인하고 model_pairs.yaml 을 갱신할 것\n{out}"
    )
