# =====================================================================
# 近距離フェーズ
# =====================================================================


import sys
import types
import math
import importlib.util

# =====================================================================
# 仮想ローバー (物理シミュレーション)
# =====================================================================

class World:
    def __init__(self, cone_bearing_deg=100.0, cone_distance_m=6.0,
                 rotate_locked=False, forward_locked=False):
        self.heading = 0.0                    # 機体方位 [deg]
        self.cone_bearing = cone_bearing_deg   # コーンの絶対方位 [deg]
        self.cone_distance = cone_distance_m   # コーンまでの距離 [m]
        self.rotate_locked = rotate_locked     # 旋回スタック状態
        self.forward_locked = forward_locked   # 直進スタック状態
        self.moving_accel = 0.0                # 直近の線形加速度ノルム
        self.log = []

    # 視野 60deg, ±30deg 以内に居れば捕捉
    def relative_bearing(self):
        return (self.cone_bearing - self.heading + 180.0) % 360.0 - 180.0

    def visible(self):
        return abs(self.relative_bearing()) <= 30.0

    def area_ratio(self):
        """距離が近いほど面積が大きくなる簡易モデル"""
        if not self.visible():
            return 0.0
        d = max(0.3, self.cone_distance)
        return min(0.9, 0.9 / (d * d))

    def do(self, direction, duration):
        self.log.append((direction, round(duration, 3)))
        if direction in ('d', 'a', 'q', 'e'):
            if self.rotate_locked:
                self.moving_accel = 0.02
                return
            deg = 120.0 * duration           # 120 deg/s 相当
            self.heading += deg if direction in ('d', 'e') else -deg
            self.heading %= 360.0
            self.moving_accel = 1.5
        elif direction == 'w':
            if self.forward_locked:
                self.moving_accel = 0.02
                return
            self.cone_distance = max(0.2, self.cone_distance - 1.2 * duration)
            self.moving_accel = 1.5
        elif direction == 's':
            if self.forward_locked:
                # 後進で脱出できる想定
                self.forward_locked = False
            self.cone_distance = min(30.0, self.cone_distance + 1.2 * duration)
            self.moving_accel = 1.5


WORLD = World()

# =====================================================================
# 偽 make_csv
# =====================================================================
fake_csv = types.ModuleType("make_csv")
fake_csv.print = lambda t, d: None
sys.modules["make_csv"] = fake_csv

# =====================================================================
# 偽 motordrive
# =====================================================================
fake_md = types.ModuleType("motordrive")


class _FakeGPIO:
    @staticmethod
    def output(pin, val):
        pass


fake_md.GPIO = _FakeGPIO
fake_md.PIN_LED = 5
fake_md.setup_motors = lambda: None
fake_md.setup_gpio = lambda: None
fake_md.stop = lambda: None
fake_md.cleanup = lambda: None


def _fake_move(direction, power, duration, is_inverted=False, enable_stack_check=True):
    import time as _t
    if is_inverted:
        direction = {'w': 's', 's': 'w', 'a': 'a', 'd': 'd', 'q': 'e', 'e': 'q'}.get(direction, direction)
    WORLD.do(direction, duration)
    # 加速度監視スレッドが判定できるよう, 実時間を圧縮して消費する
    _t.sleep(min(duration, 0.5))
    return 0


fake_md.move = _fake_move
sys.modules["motordrive"] = fake_md

# =====================================================================
# 偽 bno055
# =====================================================================
fake_bno_mod = types.ModuleType("bno055")


class FakeBNO:
    def begin(self, mode=None):
        return True

    def euler(self):
        return [WORLD.heading, 0.0, 0.0]

    def gyroscope(self):
        return [0.0, 0.0, WORLD.moving_accel]

    def magnetometer(self):
        return [10.0, 5.0, -30.0]

    def linear_acceleration(self):
        return [WORLD.moving_accel, 0.0, 0.0]

    def gravity(self):
        return [0.0, 0.0, 9.8]


fake_bno_mod.BNO055 = FakeBNO
sys.modules["bno055"] = fake_bno_mod

# =====================================================================
# 偽 camera
# =====================================================================
fake_cam_mod = types.ModuleType("camera")


class _FakeFrame:
    shape = (480, 640, 3)


class FakeCamera:
    def __init__(self, *a, **kw):
        pass

    def close(self):
        pass

    def capture_and_detect(self, is_inverted=False):
        frame = _FakeFrame()
        if not WORLD.visible():
            return frame, 0.0, 0, 0.0

        rel = WORLD.relative_bearing()          # + = コーンが右
        x_percent = max(-0.5, min(0.5, rel / 60.0))
        area_ratio = WORLD.area_ratio()
        red_area = area_ratio * 640 * 480

        if area_ratio > 0.3:
            order = 4
        elif -0.25 <= x_percent <= 0.25:
            order = 1
        elif x_percent > 0.25:
            order = 2
        else:
            order = 3
        return frame, x_percent, order, red_area


fake_cam_mod.Camera = FakeCamera
sys.modules["camera"] = fake_cam_mod

# =====================================================================
# SC-30_kinkyori_flow.py の読み込み
# =====================================================================
spec = importlib.util.spec_from_file_location("sc30", "SC-30_kinkyori_flow.py")
sc30 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sc30)

# シミュレーションを速くするためタイミング定数を短縮
sc30.STACK_ACCEL_DURATION = 0.3
sc30.STACK_IGNORE_HEAD_SEC = 0.05
sc30.PHASE_TIMEOUT_SEC = 25.0
sc30.LOOP_SLEEP = 0.0
sc30.INVERT_CHECK_INTERVAL = 999.0


def run_case(name, world, expect_goal=True):
    global WORLD
    WORLD = world
    # WORLD を偽モジュール側にも反映
    for mod_globals in (globals(),):
        mod_globals["WORLD"] = world

    print("\n" + "=" * 70)
    print(f"CASE: {name}")
    print("=" * 70)

    rover = sc30.SC30RoverHardware()
    goal = sc30.sc30_kinkyori_phase(rover)
    rover.close()

    print(f"\n>>> 結果: goal={goal} (期待={expect_goal}), "
          f"最終距離={world.cone_distance:.2f}m, heading={world.heading:.1f}deg")
    print(f">>> モーター指令回数={len(world.log)}")
    assert goal == expect_goal, f"FAIL: {name}"
    print(f">>> PASS: {name}")
    return world


if __name__ == "__main__":
    import logging
    logging.getLogger().setLevel(logging.WARNING)  # 出力を絞る

    # --- ケース1: 正常系。コーンは真横100deg, 6m先 -> 30度旋回で探索し接近 ---
    run_case("正常系 (探索 -> 中心合わせ -> 接近 -> ゴール)",
             World(cone_bearing_deg=100.0, cone_distance_m=6.0), expect_goal=True)

    # --- ケース2: 最初から正面, 至近 -> 即ゴール ---
    run_case("即ゴール (正面 かつ 至近)",
             World(cone_bearing_deg=0.0, cone_distance_m=1.0), expect_goal=True)

    # --- ケース3: 直進スタック -> 解除動作後にゴール ---
    run_case("直進スタック -> 解除 -> ゴール",
             World(cone_bearing_deg=5.0, cone_distance_m=8.0, forward_locked=True),
             expect_goal=True)

    # --- ケース4: 旋回スタック (回っても向きが変わらない) -> タイムアウト ---
    #     recovery の後進でも回れないので永久にコーンを見つけられない = 想定通り
    w = World(cone_bearing_deg=180.0, cone_distance_m=8.0, rotate_locked=True)
    sc30.PHASE_TIMEOUT_SEC = 8.0
    run_case("旋回スタック検知 (向きが変化しない)", w, expect_goal=False)
    rot_recovery = [d for d, t in w.log if d == 's']
    print(f">>> スタック解除の後進回数 = {len(rot_recovery)} (1回以上ならスタック検知が動作)")
    assert len(rot_recovery) >= 1, "FAIL: 旋回スタック検知が発火していない"

    print("\n" + "=" * 70)
    print("ALL CASES PASSED")
    print("=" * 70)
