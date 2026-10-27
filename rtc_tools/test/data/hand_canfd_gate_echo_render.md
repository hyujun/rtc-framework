## G-3 verdict: **PASS**

| check | verdict | value | threshold | note |
|---|---|---|---|---|
| `a.duration` | PASS | 300.0 s, 150001 rows | >= 300 s |  |
| `a.single_run` | PASS | run_id [0] | exactly one run_id |  |
| `a.tick_gaps` | PASS | 0 | 0 |  |
| `b.json_link_ok` | PASS | link_ok=True failure_detected=False | link_ok; a detector latch only for a link reason |  |
| `b.log_link_down` | PASS | 0 line(s) | 0 |  |
| `b.bus_silent_req` | PASS | max 1 consecutive REQ without a complete RESP set | < 50 |  |
| `b.topic_false` | PASS | 0 false sample(s) | 0 |  |
| `c.cycle_p99` | PASS | 500 us (max 500, >= limit 0, >= period 0) | < 1818 us (period / 1.1, period 2000 us) |  |
| `d.resp_set_timeout` | PASS | 3 / 150000 = 0.0020% | < 0.1% (= 150.0 ± 20.2 of 150000) |  |
| `d.echo_timeout` | PASS | 0 / 10 = 0.0000% | < 0.1% (= 0.0 ± 0.2 of 10) |  |
| `e.estop` | PASS | Δt(first zero CMD -> last echo) = 1.500 ms, candump-to-candump | 1 pair, both echoes, < 2 ms |  |

### Supporting figures

| figure | value | expected |
|---|---|---|
| protocol / transport | 2 / canfd | 2 / canfd |
| recv_timeout_ms (as run) | 0.9 | D42: 2 x T < period |
| total_cycles / timing rows | 150000 / 150001 | about equal |
| median tick period | 2000 us | 2000 us |
| t_total_us p50 / p99 / max | 500 / 500 / 500 | — |
| jitter_us p99 / max | 3 / 3 | — |
| aggregate (recv_timeout + recv_error) / total_cycles | (3 + 0) / 150000 = 0.0020% | upper bound only (up to 2 per cycle) |
| canfd.tx_eagain / tx_enobufs / tx_enetdown / tx_other | 0 / 0 / 0 / 0 | 0 |
| canfd.bus_off / error_frames | 0 / 0 | 0 (bus errors are outside error_mask) |
| canfd.echo_payload_mismatch | 0 | 0 |
| assembler sets_completed / sets_abandoned | 149997 / 3 | abandoned ~ timeouts |
| assembler bad_index / bad_length / duplicate / outside_set | 0 / 0 / 0 / 0 | 0 |
| stale_drained / cmd_mismatch / mode_mismatch | None / None / None | 0 |
| feedforward_size_mismatch / nonfinite | None / None | 0 |
| bus frames (parsed / unparsed lines / error frames) | 111 / 0 / 0 | errors 0 |
| bus CMD / ECHO / REQ / RESP / other-node | 24 / 24 / 16 / 47 / 0 | — |
| REQ with DLC > 0 | 0 | 0 (D47: DLC 0) |
| RESP sets complete / incomplete | 15 / 1 | — |
| REQ -> RESP frame 2 round trip, all ticks (bus, lower bound; a write tick's includes the echo handling since #919) | n=15 p50=0.500 p99=0.580 max=0.580 ms | — |
| REQ -> RESP frame 2 round trip, read-only ticks (bus, lower bound; the H1-F06 baseline) | n=5 p50=0.500 p99=0.500 max=0.500 ms | — |
| CMD frame 0 -> last echo round trip (bus, lower bound) | n=11 p50=0.770 p99=0.770 max=0.770 ms | — |
| CMD frame 0 -> RESP frame 2, write ticks (bus, lower bound) | n=10 p50=0.660 p99=1.000 max=1.000 ms | — |
| write attempts (JSON once-per-attempt counters) / bus write pairs | 10 / 11 | about equal |
| E-Stop frames (t rel. to CMD F1) | CMD F0 +0.100 ms, echo F1 +1.400 ms, echo F0 +1.500 ms | — |
