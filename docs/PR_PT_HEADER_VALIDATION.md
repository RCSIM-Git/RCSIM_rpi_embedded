# PR draft: Fix Pure Pursuit exception and invalid PT fragment handling

## Problem and impact

Pure Pursuit accesses undefined `self.logger` on every planning call, raising `AttributeError` before returning steering and throttle. Using the existing module logger restores the calculation without changing control formulas or thresholds.

A CRC-valid path packet with `index=1,total=1` raises `KeyError` during reassembly and escapes the command callback. A changed total for a pending message can also cause premature assembly or missing-fragment errors.

## Changes

- Use the module logger in Pure Pursuit. Add real LocalPlanner/costmap regressions for straight driving, mirrored turns and stopping near a LiDAR obstacle.
- Reject `total=0` and `index>=total` before allocating a buffer.
- Reject changed totals for pending message IDs before changing payloads or receipt times.
- Preserve valid out-of-order delivery, duplicate fragments and waypoints split across packets.
- Update README documentation in Polish and English and the existing TODO.
- Update outer workspace CHANGELOG.md and CHANGELOG_EN.md; those files already contain user changes and must be reviewed separately from this embedded repository.

## Validation

From `rpi_project_source`, using the outer workspace `.venv/Scripts/python.exe`:

```text
python -m pytest -q tests/core/test_binary_path_assembler.py tests/core/test_command_dispatcher.py
python -m compileall -q core/binary_path_assembler.py
python -m pytest -q tests/test_local_planner_fusion.py::TestPurePursuitIntegration tests/core/test_binary_path_assembler.py
python -m compileall -q modules/planners/pure_pursuit_planner.py tests/test_local_planner_fusion.py
```

- New regressions: 6 passed; original implementation: 5 failed, 1 passed.
- Pure Pursuit integration regressions: 3 failed with the original implementation, 3 passed after the logger fix. Together with PT regressions: 9 passed.
- Broader autonomy run (`tests/test_local_planner_fusion.py`, `tests/logic/test_pure_pursuit.py`, PT regressions): 11 passed, 6 failures in existing tests (four obsolete constructor calls and two NavigationManager expectations). The reviewer reproduced all six failures with the HEAD Pure Pursuit module loaded in memory, without changing files.
- Combined suite: 10 passed, 1 failed (`test_failsafe_recovery`). The same failure was reproduced with the original HEAD assembler loaded in memory without modifying files.
- Python bytecode compilation and diff whitespace checks passed.
- Review found no blocking issues. No Raspberry Pi hardware or Docker image build was validated on this Windows host.

## Publication status

Local draft only. No PR template exists in the embedded repository or outer `.github`; this draft uses problem, changes and validation sections. No commit, push or remote PR was created because the outer workspace contains unrelated user changes, including both changelogs and other repositories.
