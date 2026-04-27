# `.github/actions/` — Composite actions for ROS 2 Advanced CI

Reusable building blocks invoked by [`../workflows/ros2-advanced-ci.yml`](../workflows/ros2-advanced-ci.yml).
Each action is self-contained and documented in its own `action.yml` (top-level `description:`).

| Action | Purpose | Used by |
|--------|---------|---------|
| [`setup-rtc-env`](setup-rtc-env/action.yml) | ROS 2 distro/tooling + apt cache + colcon upgrade + numpy fix | every job |
| [`build-isolated-deps`](build-isolated-deps/action.yml) | Build & cache fmt 11 / mimalloc / aligator from `deps.repos` → publish artifact | `build-deps` (one-shot) |
| [`colcon-build`](colcon-build/action.yml) | `colcon build --packages-up-to <pkgs>` with deps prepend + failure-log artifact | `build-test`, `clang-tidy`, `codeql` |
| [`colcon-test-report`](colcon-test-report/action.yml) | `colcon test` + `GITHUB_STEP_SUMMARY` table + failure-log artifact | `build-test`, `python-test` |

## Conventions

- Each action declares all knobs as explicit `inputs:` — no implicit env-var coupling.
- `artifact-suffix` input on `colcon-build` / `colcon-test-report` MUST be unique
  per job to avoid `actions/upload-artifact` name collisions.
- `deps-install-path` is the empty string when a job does not need isolated
  deps (e.g., python-only test, cppcheck lint).
- `if: failure()` artifacts have 7-day retention (debug only). The deps cache
  and dep artifact use 1-day retention (cross-job same-run only).
- ROS distro defaults to `jazzy` (single distro per
  [`agent_docs/ci-rewrite-plan.md`](../../agent_docs/ci-rewrite-plan.md) D-3).

## When to add a new action

Only when the same multi-step block appears in ≥2 jobs. Single-use steps stay
inline in the workflow yaml.
