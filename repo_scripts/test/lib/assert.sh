#!/bin/bash
# assert.sh — shared PASS/FAIL tally for the repo_scripts shell tests: pass / fail,
# expect_eq and the closing summary. Sourced by test/test_*.sh, never run on
# its own (the CI step globs test/test_*.sh, which this path does not match).

PASS=0
FAIL=0
FAIL_MSGS=()

fail() { FAIL=$((FAIL+1)); FAIL_MSGS+=("$1"); }
pass() { PASS=$((PASS+1)); }

expect_eq() {
  # expect_eq "label" expected actual
  local label="$1" expected="$2" actual="$3"
  if [[ "$expected" == "$actual" ]]; then
    pass
  else
    fail "[$label] expected='$expected' actual='$actual'"
  fi
}

# summary_and_exit <test name>: print the tally and every failure, then exit
# 1 if anything failed, 0 otherwise.
summary_and_exit() {
  echo
  echo "── $1 summary ──"
  echo "  PASS: $PASS"
  echo "  FAIL: $FAIL"
  if (( FAIL > 0 )); then
    printf '  %s\n' "${FAIL_MSGS[@]}"
    exit 1
  fi
  exit 0
}
