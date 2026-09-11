# -*- coding: utf-8 -*-
"""问题 1 算例复现脚本（论文 §4.5）
三个检测点对同一干扰源测向（理想读数），计算三条 ±1° 误差带之交的
凸多边形顶点、直径 D(R)、最小包围圆半径 r*，并判定直径圆能否覆盖。
运行：python q1_region_example.py
"""
import math, itertools

EPS = math.radians(1.0)
S = [(0.0, 0.0), (1000.0, 0.0), (500.0, 900.0)]   # 检测点
G = (500.0, 300.0)                                 # 干扰源真实位置（仅用于生成理想读数）


def main():
    # 1) 生成 6 个半平面（每个检测点两条边界射线，n·P >= c）
    hp, bearings = [], []
    for k, Si in enumerate(S, 1):
        a = math.atan2(G[1] - Si[1], G[0] - Si[0])
        bearings.append(math.degrees(a))
        print(f"S{k}={Si}  方位角 {math.degrees(a):.4f} deg")
        for hi in (False, True):
            phi = a + EPS if hi else a - EPS
            n = (-math.sin(phi), math.cos(phi))
            c = n[0] * Si[0] + n[1] * Si[1]
            hp.append(((-n[0], -n[1]), -c) if hi else (n, c))

    # 2) 两两求交，筛选落在全部半平面内的点
    pts = []
    for i, j in itertools.combinations(range(6), 2):
        (n1, c1), (n2, c2) = hp[i], hp[j]
        det = n1[0] * n2[1] - n1[1] * n2[0]
        if abs(det) < 1e-15:
            continue
        P = ((c1 * n2[1] - c2 * n1[1]) / det, (n1[0] * c2 - n2[0] * c1) / det)
        if all(n[0] * P[0] + n[1] * P[1] >= c - 1e-9 for (n, c) in hp):
            pts.append(P)
    uniq = []
    for P in pts:
        if not any(math.hypot(P[0] - Q[0], P[1] - Q[1]) < 1e-6 for Q in uniq):
            uniq.append(P)

    # 3) 凸包得顶点（逆时针）
    uniq.sort()
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo = []
    for P in uniq:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], P) <= 0:
            lo.pop()
        lo.append(P)
    up = []
    for P in reversed(uniq):
        while len(up) >= 2 and cross(up[-2], up[-1], P) <= 0:
            up.pop()
        up.append(P)
    hull = lo[:-1] + up[:-1]
    print(f"\n定位区域顶点数 m={len(hull)}")
    for k, P in enumerate(hull, 1):
        print(f"V{k} = ({P[0]:.2f}, {P[1]:.2f})")

    # 4) 直径：顶点对穷举
    D = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in itertools.combinations(hull, 2))
    print(f"\nD(R) = {D:.2f} m   直径圆半径 = {D/2:.2f} m")

    # 5) 最小包围圆：枚举 2 点 / 3 点候选
    r_best, who = 1e18, None
    for a, b in itertools.combinations(hull, 2):
        c = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        r = math.hypot(a[0] - c[0], a[1] - c[1])
        if all(math.hypot(p[0] - c[0], p[1] - c[1]) <= r + 1e-9 for p in hull) and r < r_best:
            r_best, who = r, ("两点", (a, b), c)
    for a, b, c3 in itertools.combinations(hull, 3):
        ax, ay = a; bx, by = b; cx_, cy_ = c3
        d = 2 * (ax * (by - cy_) + bx * (cy_ - ay) + cx_ * (ay - by))
        if abs(d) < 1e-12:
            continue
        ux = ((ax*ax+ay*ay)*(by-cy_) + (bx*bx+by*by)*(cy_-ay) + (cx_*cx_+cy_*cy_)*(ay-by)) / d
        uy = ((ax*ax+ay*ay)*(cx_-bx) + (bx*bx+by*by)*(ax-cx_) + (cx_*cx_+cy_*cy_)*(bx-ax)) / d
        r = math.hypot(ax - ux, ay - uy)
        if all(math.hypot(p[0] - ux, p[1] - uy) <= r + 1e-9 for p in hull) and r < r_best:
            r_best, who = r, ("三点", (a, b, c3), (ux, uy))
    names = {id(p): f"V{i+1}" for i, p in enumerate(hull)}
    detail = [names[id(p)] for p in who[1]] if who[0] == "三点" else ""
    print(f"r* = {r_best:.2f} m  （由{who[0]}确定 {' '.join(detail)} 圆心 {who[2]}）")

    # 6) 判定
    cover = r_best <= D / 2 + 1e-9
    print(f"Jung 上界 D/sqrt(3) = {D/math.sqrt(3):.2f} m")
    print(f"判定：r* {('<=' if cover else '>')} D/2={D/2:.2f} -> 直径圆{'能' if cover else '不能'}覆盖定位区域")


if __name__ == "__main__":
    main()
