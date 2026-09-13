# -*- coding: utf-8 -*-
"""
B 题机器狗客户端 + 问题3 策略（v2 密网格版）
==========================================
纯标准库，Python 3.8+ 直接运行，无需 pip 安装。

运行方式：
    python robot_client.py            # 连通性冒烟测试（先跑这个验证接口）
    python robot_client.py strategy   # 跑问题3 搜索-定位-清除策略（全向源，演练测试用）
    python robot_client.py strategy4  # 跑问题4 策略（混入定向源：加密网格 + 环绕补探）

操作顺序（配合模拟器）：
    1) 模拟器里点「开始问题3演练测试」
    2) 出现 5 秒倒计时，倒计时结束接口才开放
    3) 此时程序可以任意时刻启动——它会自动等接口开放再 /enter
       （你也可以先启动程序，再点开始，都行）

注意：robot_id 必须等于当前登录的参赛队号。

日志：每次运行在本目录下生成独立的 robot_log_<时间戳>.jsonl，逐条记录全部
      HTTP 请求/响应以及每次定位-清除决策（估计坐标、spread、结果），
      保证论文结果可复现、可审计，且各次测试日志互不覆盖。
"""

import json
import os
import sys
import time
import math
import urllib.request
import urllib.error

# ----------------------------- 配置 -----------------------------
BASE_URL = "http://127.0.0.1:2026"
ROBOT_ID = "202606010578"          # 当前登录的参赛队号
ARENA_ID = "default"

# 策略参数（可在演练中调）
ARENA_RADIUS = 1800.0              # 目标区域半径
GRID_STEP = 800.0                  # 检测点间距：取 800 < 最小有效接收半径 1000，保证每个源至少被 2~3 个检测点覆盖
COVER_MARGIN = 600.0               # 布点向外多扩一点，覆盖边界源（最大布点半径 = 1800+600 = 2400）
CLEAR_RADIUS = 20.0                # 清除半径
MIN_BEARINGS = 3                   # 至少几条示向度才尝试交会定位（提高估计稳健性，抑制病态交会）
SPREAD_TOL = 40.0                  # 定位置信阈值：观测最大垂距 <= 该值才去清除（偏大=更激进，失败仅浪费一次清除）
MAX_ANGLE_GAP = 360.0              # 观测方位最大空隙阈值（度）：<360 时启用"单侧几何"拦截（论文8.4改进1）。
                                   # 单侧观测下各示向线近平行，spread 极小但在射线方向系统性偏移，
                                   # 此时即使 spread 达标也暂缓清除。默认 360=关闭，与正式测试冻结版一致。

# 问题4 定向源专用（±90° 扇区、方向未知）：加密网格 + 环绕补探
P4_GRID_STEP = 500.0              # 更密网格：任一位置周围落入接收半径内的检测点更多，
                                  # 即使扇区遮挡掉约一半，仍有望获得 >=3 条有效示向度
RING_PROBE_RADIUS = 700.0         # 环绕补探半径（介于最小/最大接收半径之间，确保贴近源）
RING_PROBE_N = 12                 # 环绕补探点数：均匀覆盖 360°，必能命中 180° 扇区

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_log.jsonl")


def _log(entry):
    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    except Exception:
        pass


def set_log_path(p):
    """切换到按运行时间戳命名的独立日志文件，保证每次测试日志完整、互不覆盖。"""
    global LOG_PATH
    LOG_PATH = p


class RobotClient:
    """机器狗与模拟器的通信封装（严格按附件2 协议）。"""

    def __init__(self, base_url=BASE_URL, robot_id=ROBOT_ID):
        self.base = base_url.rstrip("/")
        self.robot_id = robot_id
        self._counter = 0
        self.virtual_time = 0.0
        self.real_remaining = None

    # ---------------- 底层：串行 + 断网重试（复用 request_id） ----------------
    def _post(self, path, payload, max_retry=3):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = urllib.request.Request(
            self.base + path,
            data=body,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )
        rid = payload.get("request_id")
        for attempt in range(1, max_retry + 1):
            # 先落盘请求再发送：即使响应丢失/进程崩溃，指令序列仍可审计（附件2 §12）。
            _log({"dir": "req", "path": path, "payload": payload, "attempt": attempt})
            try:
                with urllib.request.urlopen(req, timeout=10) as resp:
                    raw = resp.read().decode("utf-8")
                try:
                    data = json.loads(raw)
                except ValueError:
                    # 响应体不是合法 JSON：记录原始片段，不静默丢弃
                    _log({"dir": "resp", "path": path, "http": 200,
                          "error": "json_decode_failed", "raw_head": raw[:500]})
                    return {"accepted": False, "_json_decode_failed": True}
                data["_http_code"] = 200
                _log({"dir": "resp", "path": path, "http": 200, "response": data})
                return data
            except urllib.error.HTTPError as e:
                try:
                    data = json.loads(e.read().decode("utf-8"))
                except Exception:
                    data = {"accepted": False}
                data["_http_code"] = e.code
                _log({"dir": "resp", "path": path, "http": e.code, "response": data})
                return data
            except (urllib.error.URLError, TimeoutError, OSError) as e:
                _log({"dir": "netfail", "path": path, "request_id": rid,
                      "attempt": attempt, "err": str(e)})
                if attempt < max_retry:
                    time.sleep(1.0)
                    continue
                _log({"dir": "netfail_final", "path": path, "request_id": rid, "err": str(e)})
                return {"accepted": False, "_connection_failed": True, "error": str(e)}
        _log({"dir": "giveup", "path": path, "request_id": rid})
        return {"accepted": False, "_unknown": True}

    def _base(self, request_id):
        return {"arena_id": ARENA_ID, "robot_id": self.robot_id, "request_id": request_id}

    def _action(self, request_id, x, y, channel):
        p = self._base(request_id)
        p["position"] = {"x": float(x), "y": float(y)}
        p["channel"] = int(channel)
        return p

    def _next_id(self, kind):
        self._counter += 1
        return "%s-%05d" % (kind, self._counter)

    def _track(self, data):
        if data.get("accepted") is True and isinstance(data.get("virtual_time_s"), (int, float)):
            self.virtual_time = data["virtual_time_s"]
        return data

    # ---------------- 四条业务指令 ----------------
    def enter(self):
        data = self._post("/enter", self._base("enter-1"))
        if data.get("accepted") is True:
            self.real_remaining = data.get("remaining_real_duration_s")
            self.virtual_time = data.get("virtual_time_s", 0.0)
        return data

    def measure(self, x, y, channel):
        return self._track(self._post(
            "/measure", self._action(self._next_id("measure"), x, y, channel)))

    def clear(self, x, y, channel):
        return self._track(self._post(
            "/clear", self._action(self._next_id("clear"), x, y, channel)))

    def exit(self):
        return self._post("/exit", self._base("exit-%05d" % (self._counter + 1)))

    def wait_and_enter(self, timeout=300, interval=2.0):
        """轮询 /enter 直到接口开放并进入成功（接口未开放时连接会失败，自动重试）。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = self._post("/enter", self._base("enter-1"))
            if r.get("accepted") is True:
                self.real_remaining = r.get("remaining_real_duration_s")
                self.virtual_time = r.get("virtual_time_s", 0.0)
                return r
            if r.get("_connection_failed"):
                print("  接口未开放，%.0fs 后重试..." % interval)
            else:
                print("  /enter 未成功：", r)
            time.sleep(interval)
        return {"accepted": False, "timeout": True}


# ============================ 几何工具 ============================
def triangulate(obs):
    """
    obs: [(px, py, deg), ...] 同一频道的多条观测。
    最小二乘法求各示向线的交点（源位置估计）。
    返回 (x, y, spread) 或 None；spread = 各线到估计点的最大垂距（越小越可信）。
    """
    A = [[0.0, 0.0], [0.0, 0.0]]
    b = [0.0, 0.0]
    for (px, py, deg) in obs:
        r = math.radians(deg)
        nx, ny = -math.sin(r), math.cos(r)      # 示向线的单位法向
        A[0][0] += nx * nx
        A[0][1] += nx * ny
        A[1][0] += nx * ny
        A[1][1] += ny * ny
        p = px * nx + py * ny
        b[0] += p * nx
        b[1] += p * ny
    det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
    if abs(det) < 1e-9:
        return None
    x = (b[0] * A[1][1] - A[0][1] * b[1]) / det
    y = (A[0][0] * b[1] - b[0] * A[1][0]) / det
    spread = 0.0
    for (px, py, deg) in obs:
        r = math.radians(deg)
        nx, ny = -math.sin(r), math.cos(r)
        spread = max(spread, abs((x - px) * nx + (y - py) * ny))
    return x, y, spread


def angular_gap(obs, x, y):
    """观测点相对估计源 (x,y) 的方位角最大空隙（度）。

    观测点方位挤在一侧时（单侧几何），最大空隙接近 360°；分布越均匀空隙越小。
    用于辅助拦截 spread 无法察觉的单侧病态（论文 8.4 改进方向 1）。
    """
    if len(obs) < 2:
        return 360.0
    angs = sorted(math.degrees(math.atan2(py - y, px - x)) % 360.0
                  for (px, py, _) in obs)
    gap = 0.0
    for i in range(len(angs)):
        nxt = angs[i + 1] if i + 1 < len(angs) else angs[0] + 360.0
        gap = max(gap, nxt - angs[i])
    return gap


def grid_points(step=None):
    """生成覆盖圆域的检测点，并从原点开始按最近邻贪心排序。

    step: 检测点间距（米）。缺省用 GRID_STEP（问题3 全向源=800）。
          问题4 定向源需更密网格以补偿扇区遮挡，可传更小值（如 500）。
    """
    s = float(step) if step else GRID_STEP
    limit = ARENA_RADIUS + COVER_MARGIN
    n = int(limit // s) + 1
    pts = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = i * s, j * s
            if math.hypot(x, y) <= limit:
                pts.append((x, y))
    ordered = []
    cur = (0.0, 0.0)
    rem = pts[:]
    while rem:
        rem.sort(key=lambda p: (p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2)
        cur = rem.pop(0)
        ordered.append(cur)
    return ordered


def ring_probe_points(cx, cy, radius=RING_PROBE_RADIUS, n=RING_PROBE_N):
    """围绕估计点 (cx,cy) 生成 n 个均匀分布的环绕补探点，用于问题4 定向源补测。

    定向源的 ±90° 扇区覆盖半个平面；网格扫描可能恰好只从单侧采到示向度，
    导致交会几何偏病态。围绕估计点 360° 均匀布点，必能命中其覆盖扇区，
    补到足够的有效示向度。仅保留仍在覆盖圆域内的点。
    """
    limit = ARENA_RADIUS + COVER_MARGIN
    pts = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        x, y = cx + radius * math.cos(a), cy + radius * math.sin(a)
        if math.hypot(x, y) <= limit:
            pts.append((x, y))
    return pts


# ============================ 问题3 策略（v2 密网格版） ============================
def run_strategy(client, channel_max=20, grid_step=None, enable_ring_probe=False,
                 mode_label="problem3"):
    import datetime as _dt
    run_stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    set_log_path(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "robot_log_%s.jsonl" % run_stamp))
    _log({"dir": "strat", "event": "run_start", "robot_id": client.robot_id,
          "run_stamp": run_stamp, "channel_max": channel_max,
          "grid_step": grid_step or GRID_STEP, "enable_ring_probe": enable_ring_probe,
          "mode": mode_label})
    r = client.wait_and_enter(timeout=300)
    if r.get("accepted") is not True:
        print("进入失败，结束。", r)
        return
    t_enter = time.time()
    real_budget = client.real_remaining if client.real_remaining is not None else 1200.0
    print("进入成功 | 可用现实时间 %.0f 秒 | 虚拟时刻 %.1f" % (real_budget, client.virtual_time))

    obs = {ch: [] for ch in range(1, channel_max + 1)}   # 各频道的示向度观测
    cleared = set()
    last_attempt = {}   # ch -> (est_x, est_y, n_obs)：v2.1 重复清除抑制（同一估计不重复浪费清除）
    points = grid_points(grid_step)
    print("检测点数量：%d（间距 %.0f m，覆盖半径 %.0f m）"
          % (len(points), (grid_step or GRID_STEP), ARENA_RADIUS + COVER_MARGIN))

    def time_up():
        # 预留 30s 余量，避免现实时间耗尽被模拟器掐断
        return (time.time() - t_enter) > (real_budget - 30.0)

    def try_localize_and_clear(tag, min_bear=MIN_BEARINGS, tol=SPREAD_TOL):
        for ch in range(1, channel_max + 1):
            if ch in cleared or len(obs[ch]) < min_bear:
                continue
            res = triangulate(obs[ch])
            if res is None:
                continue
            x, y, spread = res
            if spread <= tol:
                # 可选：单侧几何拦截（MAX_ANGLE_GAP < 360 时启用，见文件头常量说明）。
                if MAX_ANGLE_GAP < 360.0:
                    gap = angular_gap(obs[ch], x, y)
                    if gap > MAX_ANGLE_GAP:
                        _log({"dir": "strat", "event": "clear_skip", "channel": ch,
                              "reason": "one_sided", "est": [round(x, 2), round(y, 2)],
                              "spread": round(spread, 2), "angle_gap": round(gap, 1),
                              "new_obs": len(obs[ch])})
                        continue
                # v2.1 重复清除抑制：估计点几乎没动、也没有新示向度时不重复清除。
                # 背景：问题4 演练中频道15 曾因单侧几何（定向源只暴露半边）带固定偏差，
                # 同一估计连续 32 次清除未中浪费大量虚拟时间；等新示向度修正估计后再试。
                n_now = len(obs[ch])
                prev = last_attempt.get(ch)
                if prev is not None:
                    moved = math.hypot(x - prev[0], y - prev[1])
                    if moved < 10.0 and (n_now - prev[2]) < 2:
                        _log({"dir": "strat", "event": "clear_skip", "channel": ch,
                              "reason": "same_estimate", "est": [round(x, 2), round(y, 2)],
                              "moved": round(moved, 2), "new_obs": n_now - prev[2]})
                        continue
                last_attempt[ch] = (x, y, n_now)
                cr = client.clear(x, y, ch)
                _log({"dir": "strat", "event": "clear_attempt", "tag": tag, "channel": ch,
                      "est": [round(x, 2), round(y, 2)], "spread": round(spread, 2),
                      "clear_result": cr.get("clear_result"), "vt": client.virtual_time})
                if cr.get("accepted") and cr.get("clear_result") == "success":
                    cleared.add(ch)
                    print("  [%s] 频道%2d 定位(%.0f,%.0f) 偏差%.1fm -> 清除成功 (vt=%.0f)"
                          % (tag, ch, x, y, spread, client.virtual_time))
                else:
                    print("  [%s] 频道%2d 定位(%.0f,%.0f) 偏差%.1fm -> 清除未中:%s"
                          % (tag, ch, x, y, spread, cr.get("clear_result")))

    try:
        # ---- 主扫描：密网格，逐点扫全部频道 ----
        for idx, (px, py) in enumerate(points, 1):
            if time_up():
                print("  [warn] 现实时间快到，提前结束主扫描（已扫 %d/%d 点）" % (idx - 1, len(points)))
                break
            for ch in range(1, channel_max + 1):
                if time_up():
                    break
                if ch in cleared:
                    continue
                m = client.measure(px, py, ch)
                if m.get("accepted") is not True:
                    continue
                mr = m.get("measure_result")
                if mr == "direction":
                    obs[ch].append((px, py, m["svd_deg"]))
                elif mr == "near":
                    cr = client.clear(px, py, ch)
                    _log({"dir": "strat", "event": "near_clear", "channel": ch,
                          "pos": [px, py], "clear_result": cr.get("clear_result"),
                          "vt": client.virtual_time})
                    if cr.get("accepted") and cr.get("clear_result") == "success":
                        cleared.add(ch)
                        print("  [near] 频道%2d 就地清除成功 (vt=%.0f)" % (ch, client.virtual_time))
            try_localize_and_clear("sweep%d" % idx)

        # ---- 验证轮：只补扫主扫描里出现过示向度、但尚未清除的频道 ----
        pending = [c for c in range(1, channel_max + 1)
                   if c not in cleared and len(obs[c]) >= 1]
        print("验证轮：待补扫频道 %s" % pending)
        for idx, (px, py) in enumerate(points, 1):
            if time_up():
                print("  [warn] 现实时间快到，提前结束验证轮")
                break
            for ch in pending:
                if time_up():
                    break
                if ch in cleared:
                    continue
                m = client.measure(px, py, ch)
                if m.get("accepted") and m.get("measure_result") == "direction":
                    obs[ch].append((px, py, m["svd_deg"]))
            try_localize_and_clear("verify%d" % idx)

        # ---- 定向源环绕补探（仅问题4启用）：对仍有示向度但未清除的频道 ----
        if enable_ring_probe:
            print("=== 定向源环绕补探阶段 ===")
            for ch in range(1, channel_max + 1):
                if ch in cleared or len(obs[ch]) < 1:
                    continue
                res = triangulate(obs[ch])
                if res:
                    cx, cy, _ = res
                else:
                    cx = sum(p[0] for p in obs[ch]) / len(obs[ch])
                    cy = sum(p[1] for p in obs[ch]) / len(obs[ch])
                probes = ring_probe_points(cx, cy)
                for (qx, qy) in probes:
                    if ch in cleared:
                        break
                    if time_up():
                        print("  [warn] 现实时间快到，提前结束环绕补探")
                        break
                    m = client.measure(qx, qy, ch)
                    if m.get("accepted") and m.get("measure_result") == "direction":
                        obs[ch].append((qx, qy, m["svd_deg"]))
                if ch not in cleared and len(obs[ch]) >= 2:
                    res2 = triangulate(obs[ch])
                    if res2 and res2[2] <= 80.0:
                        x, y, spread = res2
                        cr = client.clear(x, y, ch)
                        _log({"dir": "strat", "event": "clear_attempt", "tag": "ring",
                              "channel": ch, "est": [round(x, 2), round(y, 2)],
                              "spread": round(spread, 2),
                              "clear_result": cr.get("clear_result"), "vt": client.virtual_time})
                        if cr.get("accepted") and cr.get("clear_result") == "success":
                            cleared.add(ch)
                            print("  [ring] 频道%2d -> 环绕补探后清除成功 (vt=%.0f)" % (ch, client.virtual_time))
                        else:
                            print("  [ring] 频道%2d 环绕补探后仍未中:%s" % (ch, cr.get("clear_result")))

        # ---- 收尾：对仅有 2 条观测的频道放宽阈值再试一次 ----
        for ch in range(1, channel_max + 1):
            if time_up():
                break
            if ch in cleared or len(obs[ch]) < 2:
                continue
            res = triangulate(obs[ch])
            if res and res[2] <= 80.0:
                x, y, spread = res
                cr = client.clear(x, y, ch)
                _log({"dir": "strat", "event": "clear_attempt", "tag": "final",
                      "channel": ch, "est": [round(x, 2), round(y, 2)],
                      "spread": round(spread, 2),
                      "clear_result": cr.get("clear_result"), "vt": client.virtual_time})
                if cr.get("accepted") and cr.get("clear_result") == "success":
                    cleared.add(ch)
                    print("  [final] 频道%2d -> 清除成功" % ch)
    finally:
        summary = {
            "cleared": sorted(cleared),
            "uncleared": [c for c in range(1, channel_max + 1) if c not in cleared],
            "vt_end": client.virtual_time,
        }
        _log({"dir": "strat", "event": "summary", **summary})
        print("=== 结束 ===")
        print("已清除频道：", sorted(cleared))
        print("未被清除频道：", summary["uncleared"])
        print("末态虚拟时刻：%.1f" % client.virtual_time)
        try:
            client.exit()
        except Exception:
            pass


# ============================ 冒烟测试 ============================
def smoke_test():
    import datetime as _dt
    run_stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    set_log_path(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "robot_log_%s.jsonl" % run_stamp))
    c = RobotClient()
    print("连接", c.base, "| robot_id =", c.robot_id)
    r = c.wait_and_enter(timeout=300)
    print("/enter ->", r)
    if r.get("accepted") is not True:
        print("进入失败：检查模拟器是否已开始测试 / 接口是否已就绪 / 队号是否正确")
        return
    print("本局可用现实时间：", c.real_remaining, "秒")
    print("measure(300,400,ch1) ->", c.measure(300, 400, 1))
    print("measure(300,400,ch2) ->", c.measure(300, 400, 2))
    print("clear(300,0,ch3)     ->", c.clear(300, 0, 3))
    print("measure(300,0,ch2)   ->", c.measure(300, 0, 2))
    print("exit ->", c.exit())


if __name__ == "__main__":
    mode = (sys.argv[1] if len(sys.argv) > 1 else "smoke").lower()
    if mode.startswith("strategy4"):
        # 问题4：混入定向源（±90° 扇区、方向未知）。加密网格 + 环绕补探。
        run_strategy(RobotClient(), grid_step=P4_GRID_STEP, enable_ring_probe=True,
                     mode_label="problem4")
    elif mode.startswith("strat"):
        # 问题3：纯全向源
        run_strategy(RobotClient())
    else:
        print("【提示】当前是连通性冒烟测试。要跑策略请用：")
        print("  问题3：python robot_client.py strategy")
        print("  问题4：python robot_client.py strategy4")
        smoke_test()
