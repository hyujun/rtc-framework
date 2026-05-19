# cm_timing_log baseline — thread-layout-v3

본 파일은 thread-layout-v3 적용 후 `cm_timing_log.csv` 의 실측 baseline 을 머신/세션별로 archive 한다. 측정 방법과 목표는 [RT_OPTIMIZATION.md §검증 §3a](../RT_OPTIMIZATION.md) 참조.

새 측정 추가 시 아래 표 행 1개로 기록 (오래된 행은 historical 로 보존, 삭제 금지).

## Sessions

### Session 1 — 6-core dev box, sim mode

| 항목 | 값 |
|---|---|
| 측정일 | 2026-05-19 |
| 머신 | dev box, 6 physical cores (12 SMT logical) |
| 커널 | Linux 6.17.0-29-generic, PREEMPT_DYNAMIC |
| RT 권한 | rtprio=99, memlock=unlimited (limits.conf @realtime) |
| cset shield | 미적용 (auto-activate on launch 안 함) |
| Launch | `ros2 launch integrated_bringup sim.launch.py` |
| Control rate | 500 Hz |
| 측정 시간 | 90 s |
| 샘플 수 | 59,065 ticks |

**Per-tick total latency (`t_total_us`)**:

| metric | value | budget (500 Hz = 2000 µs) | 헤드룸 |
|---|---|---|---|
| mean | 45.66 µs | 2000 µs | 97.7 % |
| p50 | 43.88 µs | — | — |
| p95 | 66.50 µs | — | — |
| p99 | 88.65 µs | — | 95.6 % |
| max (post-warm) | 238.92 µs | 2000 µs | 88.0 % |

**Counters** (90 s 전체):
- overruns: **0**
- skips: **0**
- pub_drops: **0** (cap 512 SPSC)
- nrt_pub_drops: **0** (cap 16 SPSC)

**Thread layout 확인** (`ps -T`):
- rt_control: SCHED_FIFO 90, Core 2 ✓
- rt_inbound: SCHED_FIFO 70, Core 3 ✓
- rt_outbound: SCHED_FIFO 65, Core 3 (same-core ✓)
- mpc_main: SCHED_FIFO 60, Core 4 ✓
- nrt_callback / nrt_logging: SCHED_OTHER, Core 0 ✓

**관찰 / 함정**:
- max 238.92 µs 는 첫 tick init spike (mlockall page touch + 첫 RCLCPP_INFO). `--filter-outliers` 적용 시 post-warm max 는 100 µs 대로 안정.
- 6-core 는 degraded mode 이지만 sim mode 에서 cset shield 없이도 p99 < 100 µs — RT priority + jthread + 2-lane SPSC 만으로 충분히 안정.
- 실 robot / 12+ core / cset shield 적용 시 더 낮은 jitter 예상 — 향후 측정 추가 시 본 표 아래 신규 session 으로 archive.

### Session N — (template)

새 측정 추가 시 본 섹션 복사:

```markdown
### Session N — <머신 설명>, <mode>

| 항목 | 값 |
|---|---|
| 측정일 | YYYY-MM-DD |
| 머신 | <CPU model, core count> |
| 커널 | `uname -v` |
| RT 권한 | `ulimit -r` / memlock |
| cset shield | 적용 여부 + 범위 |
| Launch | <launch command> |
| Control rate | <Hz> |
| 측정 시간 | <s> |
| 샘플 수 | <ticks> |

(t_total_us 표 + counters + thread layout + 관찰)
```
