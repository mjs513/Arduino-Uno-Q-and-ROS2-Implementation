import time
from ros_sonar_py.app_utils import Bridge   # Use the same Bridge instance as LED example


class Sonar:
    def __init__(self, min_angle=0.0, max_angle=180.0, step=5.0, settle_sec=0.05):
        # Use the global Bridge instance (do NOT instantiate your own)
        self.bridge = Bridge

        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.step = float(step)
        self.settle_sec = float(settle_sec)

    def _frange(self, start, stop, step):
        x = start
        while x <= stop + 1e-6:
            yield x
            x += step

    def sweep(self):
        angles = []
        distances = []

        #sweep_start = time.time()
        #print("\n=== Starting sweep ===")

        for angle in self._frange(self.min_angle, self.max_angle, self.step):
            step_start = time.time()

            # ----------------------------------------------------
            # 1. Send servo command
            # ----------------------------------------------------
            #t0 = time.time()
            try:
                self.bridge.call("set_servo_angle", float(angle))
            except Exception as e:
                print(f"[ERROR] set_servo_angle({angle}) failed: {e}")
                print("[WARN] Aborting sweep early due to servo timeout")
                distances.append(float("inf"))
                #continue
                break
            #t1 = time.time()

            # Special case for angle 0
            if angle == 0:
                time.sleep(1.0)

            # Servo settle time
            time.sleep(self.settle_sec)

            # ----------------------------------------------------
            # 2. Read distance
            # ----------------------------------------------------
            #t2 = time.time()
            try:
                dist = self.bridge.call("get_distance_m")
            except Exception as e:
                print(f"[ERROR] get_distance_m() failed at angle {angle}: {e}")
                print("[WARN] Aborting sweep early due to distance timeout")
                break
                #dist = -1.0
            #t3 = time.time()

            # Convert to float
            dist = float(dist) if dist is not None else -1.0

            # ----------------------------------------------------
            # 3. Log timing for this step
            # ----------------------------------------------------
            #print(
            #    f"Angle {angle:6.1f} deg | "
            #    f"servo_rpc={t1 - t0:6.3f}s | "
            #    f"settle={self.settle_sec:5.3f}s | "
            #    f"dist_rpc={t3 - t2:6.3f}s | "
            #    f"step_total={t3 - step_start:6.3f}s"
            #)

            angles.append(angle)
            distances.append(dist)

            # Small pacing delay to avoid RouterBridge queue overload
            time.sleep(0.01)

        sweep_end = time.time()
        #print(f"=== Sweep complete: {sweep_end - sweep_start:.3f} seconds ===\n")

        return angles, distances

