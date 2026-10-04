"""채점 월드 생성기 — 에셋 미러를 못 받았어도 대회 조건 월드는 만든다."""
import importlib.util
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location(
    "make_check_worlds", os.path.join(ROOT, "tools", "make_check_worlds.py"))
make_check_worlds = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(make_check_worlds)

# 주최 측 apartment.wbt 에서 생성기가 고치는 자리만 남긴 작은 월드
SOURCE = """#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
TurtleBot3Burger {
  controller "tb3_teleop"
}
RedApple {
}
RedApple {
}
Pedestrian {
  name "walker"
}
"""


def _source(root):
    path = root / "external" / "PNU-TECHWEEK-260930" / "worlds" / "apartment.wbt"
    path.parent.mkdir(parents=True)
    path.write_text(SOURCE, encoding="utf-8")
    (root / "worlds").mkdir()


def test_without_the_mirror_only_the_competition_world_is_made(tmp_path, capsys):
    _source(tmp_path)
    make_check_worlds.main([], root=str(tmp_path), mirror=str(tmp_path / "없는-미러"))
    made = sorted(p.name for p in (tmp_path / "worlds").iterdir())
    assert made == ["apartment_competition_check.wbt"]
    text = (tmp_path / "worlds" / "apartment_competition_check.wbt").read_text(encoding="utf-8")
    assert 'controller "mission_check"' in text and "DEF TARGET_RED_2" in text
    assert "미러" in capsys.readouterr().out


def test_with_the_mirror_all_three_worlds_are_made(tmp_path):
    _source(tmp_path)
    mirror = tmp_path / "mirror"
    (mirror / "projects").mkdir(parents=True)
    make_check_worlds.main([], root=str(tmp_path), mirror=str(mirror))
    made = sorted(p.name for p in (tmp_path / "worlds").iterdir())
    assert made == ["apartment_check.wbt", "apartment_competition_check.wbt",
                    "apartment_nopeople_check.wbt"]
    assert "Pedestrian" not in (tmp_path / "worlds" / "apartment_nopeople_check.wbt").read_text(encoding="utf-8")
