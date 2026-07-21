# `.github/actions/` — Composite actions for ROS 2 Advanced CI

Reusable building blocks invoked by [`../workflows/ros2-advanced-ci.yml`](../workflows/ros2-advanced-ci.yml).
Each action is self-contained and documented in its own `action.yml` (top-level `description:`).

| Action | Purpose | Used by |
|--------|---------|---------|
| [`setup-rtc-env`](setup-rtc-env/action.yml) | ROS 2 distro/tooling + apt cache + colcon upgrade + numpy fix | every job (+ codeql) |
| [`build-isolated-deps`](build-isolated-deps/action.yml) | Build & cache fmt 11 / mimalloc / aligator from `deps.repos` → publish artifact | `build-deps`, `codeql` |
| [`colcon-build`](colcon-build/action.yml) | `colcon build --packages-up-to <pkgs>` with deps prepend + optional `use-ccache` launcher + failure-log artifact | `gated-test`, `coverage-cpp`, `python-test`, `codeql` |
| [`colcon-test-report`](colcon-test-report/action.yml) | `colcon test` + `GITHUB_STEP_SUMMARY` table + failure-log artifact | `gated-test`, `coverage-cpp`, `python-test` |
| [`free-disk-space`](free-disk-space/action.yml) | Delete unused pre-installed toolchains (dotnet / Android / GHC / …) before a C++ build | `gated-test`, `coverage-cpp`, `codeql` |
| [`disk-report`](disk-report/action.yml) | `df -h` + `df -i` + per-directory `du -sh` at a labelled checkpoint | `gated-test`, `coverage-cpp`, `codeql` |

## Conventions

- Each action declares all knobs as explicit `inputs:` — no implicit env-var coupling.
  Sole intentional exception: `colcon-build`'s `use-ccache: "true"` reads ccache's
  native env (`CCACHE_DIR`, `CCACHE_COMPILERCHECK`, `CCACHE_MAXSIZE`), which the caller
  job sets and caches per build flavor (RelWithDebInfo vs Debug+gcov keep separate
  `CCACHE_DIR` + cache-key prefixes so instrumented objects never cross-contaminate).
- `artifact-suffix` input on `colcon-build` / `colcon-test-report` MUST be unique
  per job to avoid `actions/upload-artifact` name collisions.
- `deps-install-path` is the empty string when a job does not need isolated
  deps (e.g., python-only test, cppcheck lint).
- `if: failure()` artifacts have 7-day retention (debug only). The deps cache
  and dep artifact use 1-day retention (cross-job same-run only).
- ROS distro defaults to `jazzy` (single distro per
  [`agent_docs/ci-rewrite-plan.md`](../../agent_docs/ci-rewrite-plan.md) D-3).
- `setup-rtc-env` pre-installs `ros2-apt-source` before `setup-ros` so that
  `setup-ros` skips its own unauthenticated `api.github.com` release lookup — that
  lookup silently yields an empty version under Actions IP rate limits and then
  404s the deb download (`exit 22`), killing the job. The pre-install step is
  best-effort (always `exit 0`); bump `ros-apt-source-fallback` when the pinned
  release stops publishing a deb for the runner's Ubuntu codename.

- Disk headroom (issue #210): every C++ build job calls `free-disk-space` right after
  checkout, then `disk-report` at three checkpoints — `after-cleanup`, `pre-build`
  (after *all* cache/artifact restores, so cold and warm runs are comparable) and
  `post-build` under `if: always()`, which is the one a `No space left on device`
  failure actually needs. Both actions are best-effort and always `exit 0`; the
  `min-free-gb` tripwire on `disk-report` only emits a `::warning`, it never fails a
  job. `codeql` passes `keep-tool-cache: "true"` because `codeql-action/init`
  unpacks its bundle into `/opt/hostedtoolcache`.
- No third-party `free-disk-space` action: the delete list is short enough to own,
  and the repo pins every external action by commit SHA — not adding one is cheaper
  than reviewing one.

## When to add a new action

Only when the same multi-step block appears in ≥2 jobs. Single-use steps stay
inline in the workflow yaml.
