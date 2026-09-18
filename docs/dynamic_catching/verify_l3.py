"""L3/L4 수학 검증 스크립트.

1) 관절 최소 도달시간 닫힌해 (L3 §4.3) vs 속도·가속 제약 선형계획 + 시간 이분 탐색
2) test_l3 이 만든 cases.txt 의 C++ 결과 대조
3) 접근축 회전벡터 오차 Jacobian (L4 §4.5) vs 유한차분

필요 패키지: numpy, scipy
실행: `./test_l3` 로 cases.txt 를 만든 뒤 같은 디렉터리에서 `python3 verify_l3.py`
"""

import os

import numpy as np
import scipy.optimize as so

# --- 1) 최소 도달시간 -------------------------------------------------------


def t_rest(D, wm, a):
    wp = np.sqrt(a * D)
    return 2 * np.sqrt(D / a) if wp <= wm else D / wm + wm / a


def t_min(q0, w0, q1, wm, a):
    """time_feasibility.hpp::tMinChecked 의 파이썬 거울."""
    if abs(w0) > wm:  # C9: 초기 상태가 속도 한계를 위반 → clamp
        w0 = np.copysign(wm, w0)
    d = q1 - q0
    if abs(d) < 1e-12 and abs(w0) < 1e-12:
        return 0.0
    s = np.sign(d) if abs(d) > 1e-12 else -np.sign(w0)
    D, w = abs(d), s * w0
    if w < 0:
        return -w / a + t_rest(D + w * w / (2 * a), wm, a)
    ds = w * w / (2 * a)
    if ds > D:
        return w / a + t_rest(ds - D, wm, a)
    wp = np.sqrt(a * D + w * w / 2)
    if wp <= wm:
        return (wp - w) / a + wp / a
    return (wm - w) / a + wm / a + (D - (wm * wm - w * w) / (2 * a) - wm * wm / (2 * a)) / wm


def lp_feasible(q0, w0, q1, wm, a, T, N=400):
    dt = T / N
    n = N + 1
    A_eq, b_eq = [], []
    r = np.zeros(n)
    r[0] = 1
    A_eq.append(r)
    b_eq.append(w0)
    r = np.zeros(n)
    r[-1] = 1
    A_eq.append(r)
    b_eq.append(0.0)
    r = np.full(n, dt)
    r[0] = r[-1] = dt / 2
    A_eq.append(r)
    b_eq.append(q1 - q0)
    A_ub, b_ub = [], []
    for k in range(N):
        r = np.zeros(n)
        r[k + 1] = 1
        r[k] = -1
        A_ub.append(r)
        b_ub.append(a * dt)
        A_ub.append(-r)
        b_ub.append(a * dt)
    res = so.linprog(
        np.zeros(n),
        A_ub=np.array(A_ub),
        b_ub=b_ub,
        A_eq=np.array(A_eq),
        b_eq=b_eq,
        bounds=[(-wm, wm)] * n,
        method="highs",
    )
    return res.status == 0


def lp_tmin(q0, w0, q1, wm, a):
    lo, hi = 0.0, 10.0
    for _ in range(30):
        m = 0.5 * (lo + hi)
        if lp_feasible(q0, w0, q1, wm, a, m):
            hi = m
        else:
            lo = m
    return hi


# --- 3) 접근축 회전벡터 오차 -------------------------------------------------


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def expso3(w):
    th = np.linalg.norm(w)
    if th < 1e-14:
        return np.eye(3)
    K = skew(w / th)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def e_axis(z, a_d, sin_eps=1e-6):
    """soft_catch_reference.hpp::axisAlignError 의 거울. exp([e]x) z == a_d."""
    m = np.cross(z, a_d)
    n = np.linalg.norm(m)
    c = float(np.dot(z, a_d))
    if n < sin_eps:
        if c > 0:
            return np.zeros(3)
        r = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
        u = np.cross(z, r)
        return np.pi * u / np.linalg.norm(u)
    return (np.arctan2(n, c) / n) * m


def J_axis(z, a_d):
    """soft_catch_reference.hpp::axisAlignJacobian 의 거울 (WORLD 각속도 기준)."""
    m = np.cross(z, a_d)
    n = np.linalg.norm(m)
    c = float(np.clip(np.dot(z, a_d), -1.0, 1.0))
    th = np.arctan2(n, c)
    if n < 1e-8:
        f, fp = 1.0 + th * th / 6.0, -1.0 / 3.0
    else:
        f, fp = th / n, (th * c / n - 1.0) / (n * n)
    return fp * np.outer(m, m) + f * (skew(a_d) @ skew(z))


if __name__ == "__main__":
    rng = np.random.default_rng(0)

    err = []
    for _ in range(40):
        q0, q1 = rng.uniform(-2, 2, 2)
        wm, a, w0 = np.pi, rng.uniform(5, 20), rng.uniform(-np.pi, np.pi)
        err.append(abs(t_min(q0, w0, q1, wm, a) - lp_tmin(q0, w0, q1, wm, a)))
    print(f"[1] python t_min vs LP: max abs err = {max(err):.2e} s (LP 격자 이산화 수준이면 통과)")

    if os.path.exists("cases.txt"):
        mx = 0.0
        with open("cases.txt") as f:
            for line in f:
                q0, w0, q1, wm, a, tc = map(float, line.split())
                mx = max(mx, abs(tc - lp_tmin(q0, w0, q1, wm, a)))
        print(f"[2] C++ tMin vs LP: max abs err = {mx:.2e} s")
    else:
        print("[2] cases.txt 없음: ./test_l3 실행 후 같은 디렉터리에서 재실행")

    worst_j, worst_e = 0.0, 0.0
    h = 1e-7
    for deg in range(1, 171):
        th = np.deg2rad(deg)
        z = np.array([0.0, 0.0, 1.0])
        a_d = np.array([np.sin(th), 0.0, np.cos(th)])
        om = rng.normal(size=3)
        fd = (e_axis(expso3(om * h) @ z, a_d) - e_axis(z, a_d)) / h
        worst_j = max(worst_j, np.abs(fd - J_axis(z, a_d) @ om).max())
        worst_e = max(worst_e, np.linalg.norm(expso3(e_axis(z, a_d)) @ z - a_d))
    print(f"[3] axis rotation-vector Jacobian vs FD: max abs err = {worst_j:.2e}")
    print(f"    exp(e_a) z == a_d residual            : max = {worst_e:.2e}")
