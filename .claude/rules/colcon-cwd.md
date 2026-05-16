---
globs: ["**/CMakeLists.txt", "**/package.xml", "**/build.sh", "**/install.sh", "repo_scripts/scripts/**", ".colcon/**", ".clangd", "merge_compile_commands.py"]
---

# Colcon CWD Rule (Scoped Stub)

이 파일은 colcon/CMake/package.xml 관련 작업 시 자동 로드되는 reminder다.

## 절대 규칙

**`colcon build` / `colcon test` / `colcon test-result` 는 반드시 colcon workspace root 에서 실행한다.**

- 정상 위치: `<rtc_ws>` (예: `~/ros2_ws/rtc_ws`)
- 금지 위치: `<rtc_ws>/src/rtc-framework/` 또는 그 하위 어디든

이유: colcon 은 cwd 기준으로 `build/` · `install/` · `log/` 트리를 만든다. repo 안에서 호출하면 그 위치에 새 트리가 생기고 — `.gitignore` 로 git tracking 은 안 되지만 — `.clangd` 의 `CompilationDatabase: build/` 가 잘못된 (stale 또는 partial) 트리를 가리키고, ws-root incremental cache 와 분리되어 추적 불가한 상태가 누적된다.

## 안전한 호출 패턴

### 1순위 (권장)
```bash
./build.sh -p <pkg>     # build.sh 가 내부에서 cd "$WORKSPACE" 자동 수행
```

### 2순위 (직접 colcon 필요 시)
```bash
cd ~/ros2_ws/rtc_ws         # 반드시 ws-root 로 이동
colcon build --packages-select <pkg> --symlink-install
colcon test --packages-select <pkg> --event-handlers console_direct+
```

### 3순위 (cwd 이동 불가 시)
```bash
colcon build \
  --base-paths ~/ros2_ws/rtc_ws/src \
  --build-base ~/ros2_ws/rtc_ws/build \
  --install-base ~/ros2_ws/rtc_ws/install \
  --packages-select <pkg>
```

## 검출

작업 후 다음이 발견되면 즉시 실수다:

```bash
ls /home/junho/ros2_ws/rtc_ws/src/rtc-framework/{build,install,log} 2>/dev/null
```

→ 하나라도 존재하면 잘못된 위치에서 colcon 이 호출됨. 삭제 후 재확인.

## Bash 호출 시 주의

`Bash` 도구는 working directory 가 persistent 하지만, 새 turn 마다 cwd 가 reset 될 수 있다. **명시적으로 `cd ~/ros2_ws/rtc_ws &&` prefix 를 매번 붙이거나, `--build-base` 절대경로 옵션을 사용**한다.

## 관련 메모리

- [feedback_build_workspace_location](file:///home/junho/.claude/projects/-home-junho-ros2-ws-rtc-ws-src-rtc-framework/memory/feedback_build_workspace_location.md)
