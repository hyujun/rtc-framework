"""catch_population_trials (#821, stage 2): what the command decides, without a sim.

A unit is a sim bringup, and a test cannot run one. What it can pin is everything
the command DECIDES around the processes — each a function of its arguments:

* the plan: which units, in which order (a run that ends early has every arm);
* the command lines of the bringup, the estimator and the driver — no overlay
  file, the arm's two modes as launch arguments;
* what a unit must read back from the controller (the shipped values with the two
  modes — held against ``run_unit.sh``'s ``EXPECT_*`` defaults) and the verdict
  word for each value that is off;
* the watch verdict while the driver throws, the retry step (a rig failure only),
  the session checks, which leftover processes are the unit's to kill.

The processes themselves are checked in real runs (the issue's completion rule).
"""

import datetime
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from integrated_bringup import catch_population_trials as cpt, catching_startup as st
from rtc_tools.utils.catch_population import Arm, ResultLayout, load_config

CONFIG = Path(__file__).resolve().parents[1] / "config"
ROBOTS = ("ur5e_p1b", "iiwa7_leap")
GRID = Arm("grid", "mpc", "segment", None)
NLP = Arm("nlp", "mpc_docking", "segment", None)


# ── the plan ─────────────────────────────────────────────────────────────────
def _chunks(tmp_path, arm, n):
    folder = tmp_path / arm / "throws"
    folder.mkdir(parents=True, exist_ok=True)
    files = [folder / f"chunk_{k:03d}.json" for k in range(n)]
    for path in files:
        path.write_text("{}")
    return files


def _plan(tmp_path, n_grid=3, n_nlp=2, **kwargs):
    chunks = {GRID.name: _chunks(tmp_path, GRID.name, n_grid)}
    chunks[NLP.name] = _chunks(tmp_path, NLP.name, n_nlp)
    counts = {path: 250 for files in chunks.values() for path in files}
    counts[chunks[GRID.name][-1]] = 140  # the last chunk of a sample is the short one
    seeds = kwargs.pop("seeds", [8210])
    return cpt.plan_units(
        [GRID, NLP], seeds, chunks, counts, ResultLayout(tmp_path).unit_dir, **kwargs
    )


def test_the_arms_are_interleaved_chunk_by_chunk(tmp_path):
    units = _plan(tmp_path)
    assert [u.name for u in units] == [
        "grid_mpc/seed_8210/chunk_000",
        "nlp_mpc_docking/seed_8210/chunk_000",
        "grid_mpc/seed_8210/chunk_001",
        "nlp_mpc_docking/seed_8210/chunk_001",
        "grid_mpc/seed_8210/chunk_002",  # the arm with fewer chunks just ends
    ]
    assert [u.n_throws for u in units] == [250, 250, 250, 250, 140]
    assert units[1].unit_dir == tmp_path / "sim" / "nlp_mpc_docking" / "seed_8210" / "chunk_000"
    assert units[1].chunk == tmp_path / "nlp_mpc_docking" / "throws" / "chunk_000.json"


def test_a_seed_is_thrown_whole_before_the_next(tmp_path):
    units = _plan(tmp_path, n_grid=2, n_nlp=2, seeds=[8210, 8211])
    assert [(u.seed, u.chunk.stem, u.arm.search) for u in units] == [
        (8210, "chunk_000", "grid"),
        (8210, "chunk_000", "nlp"),
        (8210, "chunk_001", "grid"),
        (8210, "chunk_001", "nlp"),
        (8211, "chunk_000", "grid"),
        (8211, "chunk_000", "nlp"),
        (8211, "chunk_001", "grid"),
        (8211, "chunk_001", "nlp"),
    ]


def test_max_chunks_and_the_throw_limit_cut_the_plan(tmp_path):
    units = _plan(tmp_path, max_chunks=1)
    assert [u.name for u in units] == [
        "grid_mpc/seed_8210/chunk_000",
        "nlp_mpc_docking/seed_8210/chunk_000",
    ]
    limited = _plan(tmp_path, throws_limit=200)
    assert [u.n_throws for u in limited] == [200, 200, 200, 200, 140]


# ── resume and retry ─────────────────────────────────────────────────────────
@pytest.mark.parametrize(
    "status", ["FAIL:host_busy", "FAIL:estimator not activated", "FAIL:startup"]
)
def test_run_all_shs_rig_failures_are_thrown_again(status):
    assert cpt.next_step(status, 2) == "run"
    assert cpt.next_step(status, 3) == "run"


def test_an_abnormal_start_is_a_rig_failure_too():
    assert cpt.next_step("FAIL:abnormal_start", 2) == "run"


@pytest.mark.parametrize(
    "status",
    [
        "FAIL:mirror T_freeze",
        "FAIL:trials rc=1",
        "FAIL:trials rc=124",
        "FAIL:signal",
        "FAIL:switch",
        "FAIL:sim exited",
        "FAIL:contaminated foreign file x.csv",
        "FAIL:no session directory in the launch log",
        "FAIL:host_busy and more",  # the whole status, not its start
        "FAIL:error OSError: disk",
        None,  # a unit that left no status at all
    ],
)
def test_any_other_failure_stays_where_it_failed(status):
    """A unit thrown again for a reason its own throws could cause would choose which
    outcomes are recorded (run_all.sh, #746)."""
    assert cpt.next_step(status, 2) == "stay"


def test_a_launch_that_had_to_be_killed_stays_where_it_failed():
    """The RT node's CSVs may end mid-row: a record of the unit, not a rig failure."""
    assert cpt.next_step("FAIL:launch killed", 2) == "stay"


def test_the_first_look_throws_whatever_is_not_done():
    """Starting the command again is a person's decision to throw a failed unit anew."""
    assert cpt.next_step(None, 1) == "run"
    assert cpt.next_step("FAIL:trials rc=1", 1) == "run"
    assert cpt.next_step("FAIL:host_busy", 1) == "run"
    for attempt in (1, 2, 3):
        assert cpt.next_step("DONE", attempt) == "done"


def test_the_retry_pattern_is_run_all_shs_with_the_abnormal_start():
    text = (CONFIG.parent / "tools" / "catching_eval" / "run_all.sh").read_text()
    assert "'^FAIL:(host_busy|estimator not activated|startup)$'" in text
    assert cpt.RIG_FAILURES == (
        "host_busy",
        "estimator not activated",
        "startup",
        "abnormal_start",
    )


def test_a_failed_unit_is_moved_to_the_first_free_fail_name(tmp_path):
    unit = tmp_path / "chunk_003"
    assert cpt.failed_name(unit) == tmp_path / "chunk_003.fail1"
    (tmp_path / "chunk_003.fail1").mkdir()
    (tmp_path / "chunk_003.fail2").mkdir()
    assert cpt.failed_name(unit) == tmp_path / "chunk_003.fail3"


def test_a_unit_done_for_another_throw_count_or_file_is_not_this_runs(tmp_path):
    chunk = tmp_path / "chunk_000.json"
    chunk.write_text('{"throws": []}')
    unit = tmp_path / "unit"
    unit.mkdir()
    spec = cpt.UnitSpec(GRID, 8210, chunk, unit, 250)
    sha = cpt.sha256_file(chunk)
    (unit / "conditions.txt").write_text(f"n: 250\nthrows_file_sha256: {sha}\n")
    assert cpt.done_for_another_run(unit, spec) is None
    (unit / "conditions.txt").write_text(f"n: 5\nthrows_limit: 5\nthrows_file_sha256: {sha}\n")
    assert "n=5" in cpt.done_for_another_run(unit, spec)
    (unit / "conditions.txt").write_text("n: 250\nthrows_file_sha256: " + "0" * 64 + "\n")
    assert "another file" in cpt.done_for_another_run(unit, spec)


# ── command lines ────────────────────────────────────────────────────────────
def test_the_bringup_takes_the_arm_as_launch_arguments_and_no_overlay():
    command = cpt.launch_command("ur5e_p1b", NLP, viewer=True)
    assert command[:4] == ["ros2", "launch", "integrated_bringup", "sim_ur5e_p1b.launch.py"]
    assert set(command[4:]) == {
        "enable_viewer:=true",
        "viewer_input_enabled:=false",
        "use_cpu_affinity:=false",
        "enable_mpc:=true",
        "sim_lanes:=true",
        "catch_search_mode:=nlp",
        "catch_segment_mode:=mpc_docking",
        "max_log_sessions:=60",
    }
    assert not [word for word in command if "sim_overlay" in word]
    headless = cpt.launch_command("iiwa7_leap", GRID, viewer=False)
    assert headless[3] == "sim_iiwa7_leap.launch.py"
    assert "enable_viewer:=false" in headless
    assert {"catch_search_mode:=grid", "catch_segment_mode:=mpc"} <= set(headless)


def test_a_unit_whose_sim_takes_world_keys_from_its_viewer_may_not_throw():
    """#822 D7: the parameter is read off the sim; the banner only says it with a viewer."""
    banner = cpt.VIEWER_INPUT_BANNER + "\n"
    assert cpt.viewer_input_failures(False, banner, viewer=True) == []
    assert cpt.viewer_input_failures(False, "", viewer=False) == []
    assert cpt.viewer_input_failures(True, banner, viewer=True) == ["viewer_input"]
    assert cpt.viewer_input_failures(None, "", viewer=False) == ["viewer_input"]  # undeclared
    assert cpt.viewer_input_failures(False, "", viewer=True) == ["viewer_banner"]


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_bringup_names_arguments_the_robots_launch_declares(robot):
    """`ros2 launch` ignores an argument the file does not declare."""
    text = (CONFIG.parent / "launch" / cpt.launch_file(robot)).read_text()
    for word in cpt.launch_command(robot, NLP, viewer=True)[4:]:
        name = word.split(":=")[0]
        assert f'"{name}"' in text or name in ("catch_search_mode", "catch_segment_mode"), name
    assert "SEARCH_ARGUMENT" in text and "SEGMENT_ARGUMENT" in text


def test_the_estimator_is_started_from_its_own_workspace():
    command = cpt.estimator_command(
        Path("/ws/ball"),
        Path("/ws/ball/p.json"),
        8210,
        producer_revision="9005aab",
        simulator_revision="unknown",
    )
    assert command[:2] == ["bash", "-c"]
    script = command[2]
    # Sourced in that shell only, and the package must then come from that workspace.
    assert 'source "$1/install/local_setup.bash"' in script
    assert "ros2 pkg prefix ball_perception_sim" in script and "WRONG_PREFIX" in script
    assert 'exec ros2 launch ball_perception_sim sim_estimator.launch.py "${@:2}"' in script
    assert command[3:] == [
        "estimator",
        "/ws/ball",
        "profile_path:=/ws/ball/p.json",
        "producer_revision:=9005aab",
        "simulator_revision:=unknown",
        "simulator_seed:=8210",
    ]
    assert subprocess.run(["bash", "-n", "-c", script], check=False).returncode == 0


def test_the_estimators_shell_drops_this_workspace_before_it_sources_its_own():
    """The shell inherits this workspace's prefixes; a package of both would resolve here."""
    script = cpt.estimator_command(
        Path("/ws/ball"), Path("/p.json"), 1, producer_revision="x", simulator_revision="y"
    )[2]
    unset = script.index("unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH")
    underlay = script.index('source "/opt/ros/$ROS_DISTRO/setup.bash"')
    path_back = script.index('PATH="$P0:$PATH"')
    own = script.index('source "$1/install/local_setup.bash"')
    # The caller's PATH goes back in front of the underlay's bin, before the workspace's.
    assert script.index("P0=$PATH") < unset < underlay < path_back < own


def test_a_viewer_banner_printed_after_the_mirror_is_waited_for(tmp_path, monkeypatch):
    """The viewer's thread prints it once its window is up, not when the controller is
    ready (#822 review): a slow display once lost a unit to FAIL:mirror viewer_banner."""
    log = tmp_path / "launch.log"
    log.write_text("ready\n")
    late = "ready\n" + cpt.VIEWER_INPUT_BANNER + "\n"
    monkeypatch.setattr(cpt, "time", SimpleNamespace(sleep=lambda s: log.write_text(late)))
    assert cpt.wait_for_text(log, cpt.VIEWER_INPUT_BANNER, 5) == late
    log.write_text("ready\n")
    slept = []
    monkeypatch.setattr(cpt, "time", SimpleNamespace(sleep=slept.append))
    assert cpt.wait_for_text(log, cpt.VIEWER_INPUT_BANNER, 3) == "ready\n"
    assert slept == [1, 1, 1]


def test_the_estimators_shipped_profile_is_found_in_either_install(tmp_path):
    isolated = tmp_path / "install" / "ball_perception_sim" / "share" / "ball_perception_sim"
    assert cpt.estimator_profile(tmp_path) == isolated / "config" / "sim_profile.catching.json"
    merged = tmp_path / "install" / "share" / "ball_perception_sim" / "config"
    merged.mkdir(parents=True)
    (merged / "sim_profile.catching.json").write_text("{}")
    assert cpt.estimator_profile(tmp_path) == merged / "sim_profile.catching.json"


def test_a_profile_is_the_judged_estimator_only_with_the_judged_horizon(tmp_path):
    timing = SimpleNamespace(prediction_horizon_s=1.0)
    profile = tmp_path / "profile.json"
    profile.write_text('{"prediction": {"horizon_s": 1.0, "step_s": 0.05}}')
    assert cpt.profile_refusal(profile, timing) is None
    profile.write_text('{"prediction": {"horizon_s": 1}}')  # an integer is the same horizon
    assert cpt.profile_refusal(profile, timing) is None
    for text, said in (
        ('{"prediction": {"horizon_s": 1.2}}', "predicts 1.2 s ahead"),
        ('{"prediction": {"horizon_s": "1.0"}}', "predicts '1.0' s ahead"),
        ('{"prediction": {"horizon_s": true}}', "predicts True s ahead"),
        ('{"other": 1}', "states no prediction.horizon_s"),
        ("[1, 2]", "states no prediction.horizon_s"),
    ):
        profile.write_text(text)
        reason = cpt.profile_refusal(profile, timing)
        assert said in reason and str(profile) in reason, text
    profile.write_text('{"prediction": {"horizon_s": 1.2}}')
    assert "timing.prediction_horizon_s 1.0" in cpt.profile_refusal(profile, timing)
    missing = cpt.profile_refusal(tmp_path / "no_such.json", timing)
    assert "states no prediction.horizon_s" in missing


def test_the_driver_throws_the_chunk_file_under_the_configured_rules():
    command = cpt.driver_command(
        Path("/out/trials"),
        "ur5e_p1b",
        Path("/r/chunk_000.json"),
        "nlp_mpc_docking",
        limit=None,
        end_on_ball_low=True,
        host_watch="abort",
    )
    assert command == [
        "ros2",
        "run",
        "integrated_bringup",
        "catching_sim_trials",
        "/out/trials",
        "--profile",
        "ur5e_p1b",
        "--throws-file",
        "/r/chunk_000.json",
        "--host-watch",
        "abort",
        "--arm",
        "nlp_mpc_docking",
        "--end-on-ball-low",
    ]
    smoke = cpt.driver_command(
        Path("/out/trials"),
        "iiwa7_leap",
        Path("/r/chunk_000.json"),
        "grid_mpc",
        limit=5,
        end_on_ball_low=False,
        host_watch="warn",
    )
    assert smoke[smoke.index("--limit") + 1] == "5"
    assert "--end-on-ball-low" not in smoke and smoke[smoke.index("--host-watch") + 1] == "warn"


def test_the_drivers_command_line_is_one_the_driver_parses(tmp_path):
    from integrated_bringup.catching_sim_trials import parse_args
    from rtc_tools.analysis.catching_throw_list import write_throw_list

    chunk = tmp_path / "chunk_000.json"
    write_throw_list(chunk, [{"throw_id": 1, "pos": [2.0, 0.0, 1.0], "vel": [-3.0, 0.0, 4.0]}])
    command = cpt.driver_command(
        tmp_path / "trials",
        "ur5e_p1b",
        chunk,
        "grid_mpc",
        limit=5,
        end_on_ball_low=True,
        host_watch="abort",
    )
    args = parse_args(command[4:])
    assert args.throws_file == str(chunk) and args.limit == 5 and args.end_on_ball_low
    assert args.host_watch == "abort" and args.arm == "grid_mpc"


def test_the_driver_gets_run_unit_shs_time():
    assert cpt.driver_timeout_s(250) == 250 * 30 + 120
    text = (CONFIG.parent / "tools" / "catching_eval" / "run_unit.sh").read_text()
    assert "timeout $((NT * 30 + 120))" in text


def test_the_switch_is_read_off_the_service_reply():
    assert cpt.switch_command()[:5] == [
        "ros2",
        "service",
        "call",
        "/rtc_cm/switch_controller",
        "rtc_msgs/srv/SwitchController",
    ]
    request = yaml.safe_load(cpt.switch_command()[5])
    assert request["activate_controllers"] == ["demo_catching_controller"]
    assert request["deactivate_controllers"] == ["demo_joint_controller"]
    ok = "response:\nrtc_msgs.srv.SwitchController_Response(ok=True, message='switched')\n"
    assert cpt.switched(ok)
    assert not cpt.switched("rtc_msgs.srv.SwitchController_Response(ok=False, message='x')")
    assert not cpt.switched("waiting for service to become available...\n")


# ── what the controller must have loaded ─────────────────────────────────────
def _expect(robot, arm):
    folder = CONFIG / robot
    return cpt.shipped_expectation(folder, folder / "mujoco_simulator.yaml", arm)


# run_unit.sh's defaults: EXPECT_COMMIT per robot, EXPECT_TARM, the literal `True` of
# the lead, EXPECT_CLIK, EXPECT_BALL, EXPECT_NPRE per mode (6 / 9), EXPECT_DTPRE, the
# mpc planner's `n_nodes: 7`.
@pytest.mark.parametrize(("robot", "commit"), [("ur5e_p1b", 0.39), ("iiwa7_leap", 0.19)])
def test_the_shipped_expectation_is_run_unit_shs_defaults(robot, commit):
    grid = _expect(robot, GRID)
    assert (grid.search, grid.segment) == ("grid", "mpc")
    assert grid.t_freeze == commit
    assert (grid.t_arm, grid.lead_enable, grid.accel_constraint) == (0.05, True, "dynamic")
    assert grid.ball_type == "tennis"
    assert (grid.n_pre_max, grid.dt_pre_s, grid.n_nodes, grid.stop_dt_s) == (6, 0.1, 7, None)
    assert grid.grid_line == "MPC segment planner approach grid: up to 6 x 0.100 s"
    assert grid.n_nodes_mirror == "planner.segment.mpc.horizon.n_nodes"
    nlp = _expect(robot, NLP)
    assert (nlp.t_freeze, nlp.t_arm, nlp.lead_enable) == (commit, 0.05, True)
    assert (nlp.n_pre_max, nlp.dt_pre_s) == (9, 0.1)
    assert nlp.grid_line == "mpc_docking segment planner ready: up to 9 x 0.100 s"
    assert nlp.n_nodes_mirror == "planner.segment.mpc_docking.stop.n_nodes"


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_docking_stop_grid_is_the_shipped_fragments(robot):
    """run_unit.sh read stop.n_nodes / stop.dt_s from the source tree's
    segment_mpc_docking.yaml; the command reads the composed controller file."""
    fragment = CONFIG / robot / "controllers" / "catching" / "segment_mpc_docking.yaml"
    stop = yaml.safe_load(fragment.read_text())["demo_catching_controller"]["catching"]
    stop = stop["planner"]["segment"]["mpc_docking"]["stop"]
    nlp = _expect(robot, NLP)
    assert (nlp.n_nodes, nlp.stop_dt_s) == (stop["n_nodes"], stop["dt_s"])


def test_a_configuration_without_an_expected_key_is_refused_by_the_keys_name(tmp_path):
    controllers = tmp_path / "controllers"
    controllers.mkdir()
    (controllers / "demo_catching_controller.yaml").write_text(
        yaml.safe_dump(
            {
                "demo_catching_controller": {
                    "catching": {"joint_cmd": {"lag": {"T_arm": 0.05, "lead_enable": True}}}
                }
            }
        )
    )
    sim = tmp_path / "mujoco_simulator.yaml"
    sim.write_text("mujoco_simulator: {ros__parameters: {projectile_ball: {ball_type: tennis}}}\n")
    with pytest.raises(ValueError, match=r"no key planner\.segment\.mpc\.approach\.n_pre_max"):
        cpt.shipped_expectation(tmp_path, sim, GRID)
    sim.write_text("mujoco_simulator: {ros__parameters: {}}\n")
    with pytest.raises(ValueError, match=r"no projectile_ball\.ball_type"):
        cpt.shipped_ball_type(sim)


def _unit_state(robot, arm):
    """A mirror and a launch log that agree with the shipped expectation of ``arm``."""
    expect = _expect(robot, arm)
    mirror = dict.fromkeys(cpt.MIRROR_NAMES)
    mirror.update(
        {
            "joint_cmd.lag.T_arm": expect.t_arm,
            "joint_cmd.lag.lead_enable": True,
            "planner.freeze.T_freeze": expect.t_freeze,
            "joint_cmd.accel_constraint": "dynamic",
            "planner.segment.mode": arm.segment,
            "planner.search.mode": arm.search,
            f"planner.segment.{arm.segment}.approach.n_pre_max": expect.n_pre_max,
            expect.n_nodes_mirror: expect.n_nodes,
        }
    )
    if arm.segment == "mpc_docking":
        mirror["planner.segment.mpc_docking.stop.dt_s"] = expect.stop_dt_s
    tag = (
        "[integrated_rt_controller-2] [INFO] [1.0] [integrated_bringup.demo_catching_controller]: "
    )
    log = "".join(
        tag + line + "\n"
        for line in (
            f"search mode: {arm.search} — …",
            f"segment mode: {arm.segment} — takes a plan with its first segment and follows",
            expect.grid_line + " before t_c, …",
            "planner enabled: wake timeout 0.100 s, budget 0.040 s",
            "supervisor: trials enabled — commit at t_c − 0.390 s",
        )
    )
    return expect, mirror, log


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("arm", [GRID, NLP], ids=lambda a: a.name)
def test_a_unit_that_shows_the_shipped_values_may_throw(robot, arm):
    expect, mirror, log = _unit_state(robot, arm)
    assert cpt.mirror_failures(mirror, "tennis", log, expect) == []


@pytest.mark.parametrize(
    ("name", "value", "word"),
    [
        ("joint_cmd.lag.T_arm", 0.0, "T_arm"),
        ("joint_cmd.lag.T_arm", None, "T_arm"),  # a parked controller declares none
        ("joint_cmd.lag.lead_enable", False, "lead"),
        ("planner.freeze.T_freeze", 0.36, "T_freeze"),
        ("joint_cmd.accel_constraint", "kinematic", "clik_form"),
        ("planner.segment.mode", "mpc", "mode_mirror"),
        ("planner.search.mode", "grid", "search_mirror"),
        ("planner.segment.mpc_docking.approach.n_pre_max", 5, "n_pre_max"),
        ("planner.segment.mpc_docking.stop.n_nodes", 7, "n_nodes"),
        ("planner.segment.mpc_docking.stop.dt_s", 0.05, "stop_dt"),
        ("planner.segment.mpc_docking.stop.dt_s", None, "stop_dt"),
    ],
)
def test_a_mirror_value_that_is_off_is_named_in_run_unit_shs_word(name, value, word):
    expect, mirror, log = _unit_state("ur5e_p1b", NLP)
    mirror[name] = value
    assert cpt.mirror_failures(mirror, "tennis", log, expect) == [word]


def test_the_arms_key_misspelled_in_the_launch_shows_as_the_shipped_modes():
    """The failure the read-back exists for: a key one level off is created, never
    read, and the controller runs the shipped pair with no warning anywhere."""
    expect, _, _ = _unit_state("ur5e_p1b", NLP)
    _, mirror, log = _unit_state("ur5e_p1b", GRID)  # what the controller then runs
    why = cpt.mirror_failures(mirror, "tennis", log, expect)
    assert why[:3] == ["mode_mirror", "search_mirror", "mode_log"]


def test_the_mpc_arms_grid_is_read_under_its_own_mirror_names():
    expect, mirror, log = _unit_state("iiwa7_leap", GRID)
    mirror["planner.segment.mpc.horizon.n_nodes"] = 5
    mirror["planner.segment.mpc.approach.n_pre_max"] = 9
    assert cpt.mirror_failures(mirror, "tennis", log, expect) == ["n_pre_max", "n_nodes"]


def test_the_ball_and_the_startup_lines_are_part_of_the_verdict():
    expect, mirror, log = _unit_state("ur5e_p1b", NLP)
    assert cpt.mirror_failures(mirror, "beanbag", log, expect) == ["ball_type"]
    assert cpt.mirror_failures(mirror, None, log, expect) == ["ball_type"]
    lines = log.splitlines(keepends=True)
    without = {
        "mode_log": [x for x in lines if "search mode" not in x],
        "approach_grid": [x.replace("up to 9 x", "up to 5 x") for x in lines],
        "no_budget_line": [x for x in lines if "planner enabled" not in x],
    }
    for word, kept in without.items():
        assert cpt.mirror_failures(mirror, "tennis", "".join(kept), expect) == [word], word
    # A controller that does not follow segments (an older binary) says so by silence.
    old = log.replace("takes a plan with its first segment", "takes a plan")
    assert cpt.mirror_failures(mirror, "tennis", old, expect) == ["not_e1f09_binary"]


def test_the_mirror_is_recorded_as_ros2_param_get_prints_it():
    """mirror.txt is parsed by the unit tools (summarize.py reads the arm off it)."""
    _, mirror, _ = _unit_state("ur5e_p1b", NLP)
    mirror["io.n_min"] = 12
    lines = cpt.mirror_text(mirror, "tennis").splitlines()
    assert (
        len(lines) == len(cpt.MIRROR_NAMES) + 1
        and lines[-1] == "ball_type: String value is: tennis"
    )
    assert "joint_cmd.lag.T_arm: Double value is: 0.05" in lines
    assert "joint_cmd.lag.lead_enable: Boolean value is: True" in lines
    assert "planner.freeze.T_freeze: Double value is: 0.39" in lines
    assert "io.n_min: Integer value is: 12" in lines
    assert "planner.search.mode: String value is: nlp" in lines
    assert "reference.omega: Parameter not set" in lines
    # The first names are run_unit.sh's, in its order.
    text = (CONFIG.parent / "tools" / "catching_eval" / "run_unit.sh").read_text()
    listed = text[text.index("for P in ") : text.index('; do\n  echo "$P: ')]
    assert tuple(listed.replace("for P in ", "").replace("\\\n", " ").split()) == cpt.MIRROR_NAMES


def test_the_units_overlay_record_spells_the_keys_the_launch_wrote():
    from integrated_bringup.catch_modes import catch_mode_overrides

    record = cpt.overrides_as_overlay(catch_mode_overrides("nlp", "mpc_docking"))
    assert record == {
        "integrated_rt_controller": {
            "ros__parameters": {
                "demo_catching_controller": {
                    "catching": {
                        "planner": {"search": {"mode": "nlp"}, "segment": {"mode": "mpc_docking"}}
                    }
                }
            }
        }
    }


# ── what the logs and the session say ────────────────────────────────────────
_RT = "[integrated_rt_controller-2] [INFO] [1.0] [integrated_rt_controller]: "
_LANE = (
    "[mujoco_simulator_node-1] [INFO] [1.0] [mujoco_simulator]: [MuJoCoSimulatorNode] ball "
    "contact lane -> /ws/logging_data/261011_2332/sim/ball_contact_lane.csv\n"
)


def test_the_session_is_the_one_the_rt_node_says_it_opened():
    log = _LANE + _RT + "Session dir: /ws/logging_data/261011_2332 (max_sessions=60)\n"
    assert cpt.session_dir_from_log(log, "/other/logging_data") == "/ws/logging_data/261011_2332"


def test_without_that_line_the_session_is_under_the_launchs_logging_root():
    assert cpt.session_dir_from_log(_LANE, "/root/logging_data") == (
        "/root/logging_data/261011_2332"
    )


def test_a_fallback_name_older_than_the_units_minute_is_not_taken():
    """Older than the minute the launch was started in: another run's session."""
    session, why = cpt.unit_session(_LANE, "/root/logging_data", "261011_2333")
    assert session is None
    assert "261011_2332, older than this unit's minute 261011_2333" in why


@pytest.mark.parametrize("minute", ["261011_2332", "261011_2331"])
def test_a_fallback_name_of_the_units_minute_or_later_is_taken(minute):
    """The launch may open its session in the minute after the one it was started in."""
    assert cpt.unit_session(_LANE, "/root/logging_data", minute) == (
        "/root/logging_data/261011_2332",
        "",
    )


def test_the_rt_nodes_line_is_taken_whatever_the_minute():
    log = _LANE + "Session dir: /ws/logging_data/261011_2330 (max_sessions=60)\n"
    assert cpt.unit_session(log, "/root", "261011_2359") == ("/ws/logging_data/261011_2330", "")


def test_a_log_that_names_no_session_gives_none():
    """An empty session path once made the workspace root the "session" to copy."""
    assert cpt.session_dir_from_log("", "/root/logging_data") is None
    assert cpt.session_dir_from_log("logging_data/ is empty\n", "/root/logging_data") is None


def test_the_startup_lines_a_unit_keeps():
    tag = (
        "[integrated_rt_controller-2] [INFO] [1.0] [integrated_bringup.demo_catching_controller]: "
    )
    log = (
        tag
        + "planner enabled: wake timeout 0.100 s, budget 0.040 s\n"
        + tag
        + "mpc_docking segment planner ready: up to 9 x 0.100 s before t_c\n"
        + tag
        + "supervisor: trials enabled — commit at t_c − 0.390 s\n"
    )
    assert cpt.planner_budget(log) == "0.040"
    assert cpt.planner_budget("planner disabled\n") is None
    assert cpt.commit_line(log) == "commit at t_c − 0.390 s"
    assert cpt.commit_line("") == ""
    assert cpt.segment_startup_lines(log) == [
        "mpc_docking segment planner ready: up to 9 x 0.100 s before t_c"
    ]


@pytest.mark.parametrize(
    ("robot", "names"),
    [
        ("ur5e_p1b", ("ur5e_state.csv", "p1b_state.csv")),
        ("iiwa7_leap", ("iiwa7_state.csv", "leap_state.csv")),
    ],
)
def test_the_state_logs_are_the_controllers_devices(robot, names):
    """run_unit.sh named them per robot (STATE_RE); the shipped `topics` say them."""
    from rtc_tools.utils.controller_config import load_controller_config

    tree = load_controller_config(
        CONFIG / robot / "controllers" / "demo_catching_controller.yaml",
        config_key="demo_catching_controller",
    )["demo_catching_controller"]
    assert cpt.state_log_names(tree) == names


def _session(tmp_path, files):
    folder = tmp_path / "261011_2332" / "controllers" / "demo_catching_controller"
    folder.mkdir(parents=True)
    for name, text in files.items():
        (folder / name).write_text(text)
    return tmp_path / "261011_2332"


STATE = ("ur5e_state.csv", "p1b_state.csv")


def test_a_units_own_session_is_clean(tmp_path):
    session = _session(
        tmp_path,
        {
            "catching_diag.csv": "t,mode,x\n0.0,1,2.0\n",
            "planner_events.csv": "t,event\n",  # a header alone is not a mismatch
            "nlp_candidates.csv": "a,b\n1,2\n",
            "ur5e_state.csv": "t,q\n0,1\n",
            "p1b_state.csv": "t,q\n0,1\n",
        },
    )
    assert cpt.session_contamination(session, STATE) is None
    # A bringup that wrote no controller log yet has nothing to refuse here.
    assert cpt.session_contamination(tmp_path / "nothing", STATE) is None


def test_a_fixture_csv_written_into_the_same_minute_is_refused(tmp_path):
    session = _session(tmp_path, {"catching_diag.csv": "t,mode,x\n0.0,1\n"})
    assert cpt.session_contamination(session, STATE) == (
        "contaminated catching_diag.csv header 3 rows 2"
    )


def test_a_file_that_is_not_the_controllers_is_refused(tmp_path):
    session = _session(tmp_path, {"catching_diag.csv": "t\n0\n", "iiwa7_state.csv": "t\n0\n"})
    assert cpt.session_contamination(session, STATE) == (
        "contaminated foreign file iiwa7_state.csv"
    )


# ── while the driver throws ──────────────────────────────────────────────────
def _look(**over):
    look = {
        "driver_rc": None,
        "elapsed_s": 60.0,
        "timeout_s": 7620.0,
        "viewer_keys": [],
        "start": (st.WAITING, None),
        "sim_alive": True,
    }
    look.update(over)
    return cpt.watch_verdict(**look)


def test_a_unit_that_throws_normally_is_let_go():
    assert _look() == ("go", None)
    assert _look(start=(st.READY, None)) == ("go", None)
    assert _look(driver_rc=0) == ("ended", None)
    assert _look(driver_rc=3) == ("ended", None)  # the driver's exit code is read after


def test_a_key_in_the_viewer_ends_the_unit_as_abnormal():
    what, why = _look(viewer_keys=["[Viewer] World gravity OFF"])
    assert what == "abnormal" and "[Viewer] World gravity OFF" in why


def test_a_flapping_start_ends_the_unit_as_abnormal():
    assert _look(start=(st.FAILED, "throw 0 went through 161 modes")) == (
        "abnormal",
        "throw 0 went through 161 modes",
    )


def test_an_abnormal_sign_outweighs_what_the_driver_returned():
    """A driver that ran to its end in a sim without gravity returns 0."""
    assert _look(driver_rc=0, viewer_keys=["[Viewer] Contacts DISABLED"])[0] == "abnormal"
    assert _look(driver_rc=1, start=(st.FAILED, "x"))[0] == "abnormal"


def test_a_sim_that_left_and_a_driver_out_of_time_end_the_unit():
    assert _look(sim_alive=False)[0] == "sim_exited"
    assert _look(elapsed_s=7620.0)[0] == "timeout"
    assert _look(elapsed_s=7619.0)[0] == "go"


def test_a_growing_log_is_read_in_whole_lines_once(tmp_path):
    log = tmp_path / "launch.log"
    tail = cpt.LogTail(log)
    assert tail.new() == ""  # not written yet
    log.write_text("one\n[sim] [Viewer] World gra")
    assert tail.new() == "one\n"
    with open(log, "a") as handle:
        handle.write("vity OFF\nthree\n")
    assert st.viewer_interference(tail.new()) == ["[Viewer] World gravity OFF"]
    assert tail.new() == ""


# ── shutdown ─────────────────────────────────────────────────────────────────
WS, BALL = "/home/u/rtc_ws/install", "/home/u/ball_ws"
PATTERNS = cpt.residual_patterns(
    {
        "mujoco_simulator_node": f"{WS}/rtc_mujoco_sim",
        "integrated_rt_controller": f"{WS}/integrated_bringup",
    },
    BALL,
)


def test_only_the_units_three_executables_are_left_to_kill():
    table = [
        (10, 1, f"{WS}/rtc_mujoco_sim/lib/rtc_mujoco_sim/mujoco_simulator_node --ros-args -r x"),
        (
            11,
            1,
            f"{WS}/integrated_bringup/lib/integrated_bringup/integrated_rt_controller --ros-args",
        ),
        (12, 1, f"{BALL}/install/ball_perception_sim/lib/ball_perception_sim/sim_estimator_node"),
        # another workspace's simulator and estimator are not this unit's
        (
            20,
            1,
            "/home/u/other_ws/install/rtc_mujoco_sim/lib/rtc_mujoco_sim/mujoco_simulator_node",
        ),
        (
            21,
            1,
            "/opt/ball/install/ball_perception_sim/lib/ball_perception_sim/sim_estimator_node",
        ),
        # a process that only NAMES one of the paths is not the executable
        (30, 1, f"tail -f {BALL}/install/ball_perception_sim/share/ball_perception_sim/x.log"),
        (
            31,
            1,
            f"/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch x {WS}/rtc_mujoco_sim/mujoco_simulator_node",
        ),
    ]
    assert cpt.residual_pids(table, PATTERNS, set()) == [10, 11, 12]


def test_the_commands_own_family_is_spared():
    """Its own command line names the estimator's workspace (--profile)."""
    table = [
        (5, 1, f"{BALL}/install/ball_perception_sim/lib/ball_perception_sim/sim_estimator_node"),
        (6, 5, f"{BALL}/install/ball_perception_sim/lib/ball_perception_sim/sim_estimator_node"),
    ]
    assert cpt.residual_pids(table, PATTERNS, {5}) == [6]


def test_only_the_units_own_sessions_are_left_to_kill():
    """Another sim of this workspace (another domain, a person's) runs the same path."""
    path = f"{WS}/rtc_mujoco_sim/lib/rtc_mujoco_sim/mujoco_simulator_node"
    table = [(10, 1, path), (11, 1, path), (12, 1, path)]
    sessions = {10: 100, 11: 200, 12: 300}
    kill = cpt.residual_pids(table, PATTERNS, set(), sessions=sessions, only_sessions={100, 300})
    assert kill == [10, 12]
    # No group of the unit: no session is the unit's, nothing is killed.
    assert cpt.residual_pids(table, PATTERNS, set(), sessions=sessions, only_sessions=set()) == []


def test_a_process_is_read_in_its_session():
    import os

    from integrated_bringup.catching_sim_trials import process_sessions

    assert process_sessions()[os.getpid()] == os.getsid(0)


# ── the free minute, the revisions, the configuration ────────────────────────
def test_a_unit_never_starts_in_the_minute_of_the_previous_launch(tmp_path, monkeypatch):
    minutes = iter(["261011_2238", "261011_2238", "261011_2239"])
    monkeypatch.setattr(cpt, "minute_stamp", lambda: next(minutes))
    stamp = tmp_path / "sim" / ".last_minute"
    stamp.parent.mkdir()
    stamp.write_text("261011_2238\n")
    slept = []
    assert cpt.wait_for_free_minute(stamp, str(tmp_path / "logging_data"), sleep=slept.append) == (
        "261011_2239"
    )
    assert slept == [2, 2] and stamp.read_text() == "261011_2239\n"


def test_a_unit_never_starts_in_a_minute_that_already_has_a_session(tmp_path, monkeypatch):
    """Whoever opened it — another evaluation in this workspace, a person's launch."""
    minutes = iter(["261011_2238", "261011_2239"])
    monkeypatch.setattr(cpt, "minute_stamp", lambda: next(minutes))
    root = tmp_path / "logging_data"
    (root / "261011_2238").mkdir(parents=True)
    slept = []
    stamp = tmp_path / "sim" / ".last_minute"  # no stamp yet: the first unit of a folder
    assert cpt.wait_for_free_minute(stamp, str(root), sleep=slept.append) == "261011_2239"
    assert slept == [2] and stamp.read_text() == "261011_2239\n"


def test_the_stamp_is_the_name_a_session_gets():
    from rtc_tools.utils.session_dir import is_session_dir_name

    assert is_session_dir_name(cpt.minute_stamp())
    assert cpt.minute_stamp() == datetime.datetime.now().strftime("%y%m%d_%H%M")


def test_a_tree_without_a_repository_has_an_unknown_revision(tmp_path):
    """An installed package has no repository: the unit records `unknown`, and runs."""
    assert cpt.revision(str(tmp_path)) == ("unknown", "unknown", "unknown")
    assert cpt.revision(str(tmp_path / "missing")) == ("unknown", "unknown", "unknown")
    assert cpt.revision(None) == ("unknown", "unknown", "unknown")


def test_a_dirty_tree_is_recorded_not_refused(tmp_path):
    def git(*words):
        subprocess.run(["git", "-C", str(tmp_path), *words], check=True, capture_output=True)

    git("init", "-q")
    git("config", "user.email", "t@example.com")
    git("config", "user.name", "t")
    (tmp_path / "a.txt").write_text("a\n")
    git("add", "a.txt")
    git("commit", "-q", "-m", "a")
    rev, dirty, files = cpt.revision(str(tmp_path))
    assert len(rev) >= 7 and (dirty, files) == ("0", "")
    (tmp_path / "a.txt").write_text("b\n")
    assert cpt.revision(str(tmp_path))[1:] == ("1", " M a.txt")


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_result_folders_configuration_must_be_the_one_given(robot, tmp_path):
    config = load_config(CONFIG / robot / "catch_population.yaml")
    copy = tmp_path / "config.yaml"
    assert cpt.stage1_mismatch(config, copy) == []  # stage 1 left no copy: nothing to hold
    text = config.path.read_text()
    copy.write_text(text)
    assert cpt.stage1_mismatch(config, copy) == []
    # `sim` is this stage's: another PC may throw other seeds, without a viewer.
    document = yaml.safe_load(text)
    document["sim"].update({"viewer": False, "estimator_seeds": [1, 2], "retries": 5})
    copy.write_text(yaml.safe_dump(document))
    assert cpt.stage1_mismatch(config, copy) == []
    document["population"]["min_pass_time_s"] = 0.6
    document["sample"]["seed"] = 822
    copy.write_text(yaml.safe_dump(document))
    assert cpt.stage1_mismatch(config, copy) == ["population", "sample"]
    copy.write_text("schema: something else\n")
    assert "cannot be read as a configuration" in cpt.stage1_mismatch(config, copy)[0]


# ── the command line ─────────────────────────────────────────────────────────
def test_the_estimators_workspace_comes_from_the_argument_or_the_environment(monkeypatch):
    base = ["--config", "c.yaml", "--out", "r"]
    monkeypatch.delenv("BALL_SIM_WS", raising=False)
    monkeypatch.delenv("PROFILE", raising=False)
    monkeypatch.delenv("EVAL_DOMAIN", raising=False)
    args = cpt.parse_args(base)
    assert args.estimator_ws is None and args.profile is None and args.domain_id == 88
    monkeypatch.setenv("BALL_SIM_WS", "/env/ws")
    monkeypatch.setenv("EVAL_DOMAIN", "42")
    assert cpt.parse_args(base).estimator_ws == "/env/ws"
    assert cpt.parse_args(base).domain_id == 42
    assert cpt.parse_args([*base, "--estimator-ws", "/arg/ws"]).estimator_ws == "/arg/ws"
    args = cpt.parse_args([*base, "--arms", "grid_mpc", "--seeds", "1", "2", "--max-chunks", "2"])
    assert (args.arms, args.seeds, args.max_chunks) == (["grid_mpc"], [1, 2], 2)
    assert not args.collect_only and not args.dry_run and args.viewer is None
    for bad in (["--max-chunks", "0"], ["--throws-limit", "0"], ["--viewer", "maybe"]):
        with pytest.raises(SystemExit):
            cpt.parse_args([*base, *bad])


def test_a_dry_run_prints_every_unit_and_what_it_would_start(tmp_path, capsys):
    units = _plan(tmp_path, n_grid=1, n_nlp=1, throws_limit=5)
    (units[0].unit_dir).mkdir(parents=True)
    (units[0].unit_dir / "status").write_text("DONE\n")
    ctx = SimpleNamespace(
        robot="ur5e_p1b",
        viewer=True,
        estimator_ws=Path("/ws/ball"),
        profile=Path("/ws/ball/p.json"),
        throws_limit=5,
        end_on_ball_low=True,
        host_watch="abort",
        revisions={"rtc_framework": ("unknown",) * 3, "ball_perception": ("9005aab", "0", "")},
    )
    cpt.print_plan(units, ctx)
    out = capsys.readouterr().out.splitlines()
    assert out[0] == "grid_mpc/seed_8210/chunk_000  n=5  status=DONE"
    assert "catch_search_mode:=grid catch_segment_mode:=mpc" in out[1]
    assert out[2].startswith("  estimator /ws/ball profile_path:=/ws/ball/p.json")
    assert "simulator_seed:=8210" in out[2] and "--limit 5" in out[3]
    assert out[4] == "nlp_mpc_docking/seed_8210/chunk_000  n=5  status=None"
