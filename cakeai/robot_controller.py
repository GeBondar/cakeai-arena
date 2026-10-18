"""Port of include/RobotController.h + src/RobotController.cpp (CakeAI-V1).

Ported line by line: states 0/1/2, spin-in-place, stuck detector, heading PID,
"extra speed", reverse, upside-down handling by marker ID, CH3 soft start.
Every std::chrono call is replaced by clock() — simulation time in seconds.
Output goes to `link.set_motors(ch1, ch2, ch3)` — the Arduino emulator here.
"""
import math

from .pid import PIDController


class RobotController:
    def __init__(self, clock, link=None, kp=0.1, ki=0.1, kd=0.1,
                 base=50.0, vmin=0.0, vmax=100.0, verbose=False):
        self.clock = clock
        self.link = link
        self.verbose = verbose
        self.pid = PIDController(clock, kp, ki, kd)
        self.ch1 = base
        self.ch2 = base
        self.ch3 = 0.0
        self.baseValue, self.minValue, self.maxValue = base, vmin, vmax
        self.markerDetected = False
        self.motionDetected = False
        self.currentOffset = 0.0
        self.krlCoefficient = 1.0
        self.kfbCoefficient = 1.0
        self.centerAfterAttack = 1
        self.centerTriggerThreshold = 150
        self.enemyTriggerThreshold = 150
        self.enemyTriggerTimeout = 2000
        self.reverseAngleThreshold = 180
        self.useReverse = 1
        self.InPlaceSpeedRotation = 0.5
        self.InPlaceSpeedWeapon = 1.0
        self.angleMinExtraSpeed = 90.0
        self.angleMaxExtraSpeed = 0.0
        self.robotState = 0
        self.lastTimeStateUpdate = 0
        self.reverse = False
        self.attack_count = 0
        self.rotatingInPlace = False
        self.positionHistory = []
        self.maxHistorySize = 30
        self.checkBackSteps = 3
        self.checkIntervalMs = 1000
        self.lastCheckTime = clock()
        self.robotIsMoving = True
        self.weaponCycleStartTime = clock()
        self.weaponCycleDurationMs = 4000
        self.ch3StartPending = False
        self.ch3StartTime = clock()
        self.ch3RampUpDurationMs = 3000
        self.ch3TargetValue = 100.0

    # ---------------------------------------------------------------- helpers
    def _now_ms(self):
        return int(self.clock() * 1000)

    def _log(self, *a):
        if self.verbose:
            print(*a)

    @staticmethod
    def getMarkerSide(markerId):
        if markerId == 0:
            return 1
        if markerId == 1:
            return 0
        return -1

    @staticmethod
    def swapInvertChannels(ch1, ch2):
        return 100.0 - ch2, 100.0 - ch1

    def getState(self):
        return self.robotState

    def setState(self, state):
        self.robotState = state
        self.lastTimeStateUpdate = self._now_ms()

    def apply_config(self, cfg):
        """cfg = dict from build/config.ini ([PanelValues] section)."""
        g = lambda k, d: cfg.get(k, d)
        self.pid.set_parameters(float(g("Kp", 0.1)), float(g("Ki", 0.1)), float(g("Kd", 0.1)))
        self.krlCoefficient = float(g("KrlCoefficient", 1.0))
        self.kfbCoefficient = float(g("KfbCoefficient", 1.0))
        self.centerAfterAttack = int(g("CenterAfterAttack", 1))
        self.centerTriggerThreshold = int(g("CenterTriggerThreshold", 150))
        self.enemyTriggerThreshold = int(g("EnemyTriggerThreshold", 150))
        self.enemyTriggerTimeout = int(g("EnemyTriggerTimeout", 2000))
        self.reverseAngleThreshold = int(g("ReverseAngleThreshold", 180))
        self.useReverse = int(g("UseReverse", 1))
        self.angleMinExtraSpeed = float(g("AngleMinExtraSpeed", 90.0))
        self.angleMaxExtraSpeed = float(g("AngleMaxExtraSpeed", 0.0))
        self.InPlaceSpeedRotation = float(g("InPlaceSpeedRotation", 0.5))
        self.InPlaceSpeedWeapon = float(g("InPlaceSpeedWeapon", 1.0))

    # ----------------------------------------------------------- Arduino I/O
    def sendToArduino(self):
        if self.link is not None:
            # static_cast<int>: truncation towards zero, as in the original
            self.link.set_motors(int(self.ch1), int(self.ch2), int(self.ch3))

    def setChannels(self, c1, c2, c3):
        clamp = lambda v: max(self.minValue, min(self.maxValue, v))
        self.ch1, self.ch2, self.ch3 = clamp(c1), clamp(c2), clamp(c3)

    def clearPositionHistory(self):
        self.positionHistory.clear()
        self.lastCheckTime = self.clock()
        self.robotIsMoving = True
        self.rotatingInPlace = False
        self.weaponCycleStartTime = self.clock()

    def reset(self):
        self.pid.reset()
        self.ch1 = self.baseValue
        self.ch2 = self.baseValue
        self.ch3 = 0.0
        self.markerDetected = False
        self.motionDetected = False
        self.currentOffset = 0.0
        self.reverse = False
        self.attack_count = 0
        self.robotState = 0
        self.rotatingInPlace = False
        self.robotIsMoving = True
        self.clearPositionHistory()
        self.ch3StartPending = False
        self.sendToArduino()

    def startCh3RampUp(self, target=100.0, duration_ms=3000):
        self.ch3TargetValue = target
        self.ch3RampUpDurationMs = duration_ms
        self.ch3StartPending = True
        self.ch3StartTime = self.clock()

    def updateCh3RampUp(self):
        if not self.ch3StartPending:
            return
        elapsed = int((self.clock() - self.ch3StartTime) * 1000)
        if elapsed >= self.ch3RampUpDurationMs:
            self.ch3 = self.ch3TargetValue
            self.ch3StartPending = False
        else:
            self.ch3 = elapsed / self.ch3RampUpDurationMs * self.ch3TargetValue

    # ------------------------------------------------------- stuck detector
    def checkAndCompareMarker(self, markerAngle, markerX, markerY):
        now = self.clock()
        elapsed = int((now - self.lastCheckTime) * 1000)
        if elapsed >= self.checkIntervalMs:
            self.positionHistory.append((markerX, markerY, markerAngle, now))
            while len(self.positionHistory) > self.maxHistorySize:
                self.positionHistory.pop(0)
            stepsBack = self.checkBackSteps
            if len(self.positionHistory) <= stepsBack:
                self.lastCheckTime = now
                self.robotIsMoving = True
                self.rotatingInPlace = False
                return True
            cur = self.positionHistory[-1]
            cmp_ = self.positionHistory[-1 - stepsBack]
            posDiff = math.hypot(cur[0] - cmp_[0], cur[1] - cmp_[1])
            rotDiff = abs(cur[2] - cmp_[2])
            while rotDiff > 180.0:
                rotDiff = abs(rotDiff - 360.0)
            if rotDiff < 10 and posDiff < 30:
                self.robotIsMoving = False
                self.rotatingInPlace = False
                self.lastCheckTime = now
                return False
            self.robotIsMoving = True
            self.rotatingInPlace = False
            self.lastCheckTime = now
            return True
        return self.robotIsMoving

    # --------------------------------------------------------------- process
    def process(self, markerAngle, centerX, centerY, markerX, markerY,
                motionX, motionY, markerId=-1):
        self.updateCh3RampUp()
        self.markerDetected = markerX >= 0 and markerY >= 0
        self.motionDetected = motionX >= 0 and motionY >= 0

        robotIsMoving = True
        if self.markerDetected:
            robotIsMoving = self.checkAndCompareMarker(markerAngle, markerX, markerY)

        if self.motionDetected:
            if self.getState() == 2:
                self.setState(1)
        else:
            self.setState(2)

        shouldRotate = False
        if not robotIsMoving and self.markerDetected:
            shouldRotate = True
        if not self.markerDetected:
            shouldRotate = True
        distance_to_center = math.hypot(centerX - markerX, centerY - markerY)
        if self.getState() == 2 and distance_to_center < self.centerTriggerThreshold:
            shouldRotate = True
        self.rotatingInPlace = shouldRotate

        if shouldRotate:
            rotationSpeed = self.InPlaceSpeedRotation
            weaponSpeed = self.InPlaceSpeedWeapon
            base_weapon = weaponSpeed
            if not robotIsMoving:
                cycle = self.weaponCycleDurationMs
                t_in = int((self.clock() - self.weaponCycleStartTime) * 1000) % cycle
                p = t_in / cycle
                mult = 1.0 - p if p < 0.5 else (p - 0.5) + 0.5
                weaponSpeed = base_weapon * mult
            else:
                self.weaponCycleStartTime = self.clock()
                weaponSpeed = base_weapon
            self.ch1 = 50 - (rotationSpeed / 2 * 100)
            self.ch2 = 50 + (rotationSpeed / 2 * 100)
            if not self.ch3StartPending:
                self.ch3 = weaponSpeed * 100
            self.currentOffset = 0.0
            self.sendToArduino()
            return

        if robotIsMoving:
            self.weaponCycleStartTime = self.clock()

        distance_to_enemy = math.hypot(motionX - markerX, motionY - markerY)
        timeDiff = self._now_ms() - self.lastTimeStateUpdate

        if self.centerAfterAttack == 1:
            if distance_to_center < self.centerTriggerThreshold and self.getState() == 0:
                self.setState(1)
                self.reverse = False
            if distance_to_enemy < self.enemyTriggerThreshold and self.getState() == 1:
                self.setState(0)
                self.attack_count += 1

        if self.getState() == 0 and timeDiff > self.enemyTriggerTimeout:
            angle_to_target = math.degrees(math.atan2(centerY - markerY, centerX - markerX))
            if self.attack_count > 0:
                self.reverse = True
        elif self.getState() == 2:
            angle_to_target = math.degrees(math.atan2(centerY - markerY, centerX - markerX))
        else:
            angle_to_target = math.degrees(math.atan2(motionY - markerY, motionX - markerX))

        corrected = markerAngle - 90.0
        if corrected > 180:
            corrected -= 360
        if corrected < -180:
            corrected += 360

        off = angle_to_target - corrected
        if off > 180:
            off -= 360
        if off < -180:
            off += 360
        # C++ compares against int/2 — integer division
        if abs(off) < self.reverseAngleThreshold // 2 and self.reverse:
            self.reverse = False
        if self.reverse:
            off += 180
            if off > 180:
                off -= 360
        self.currentOffset = off

        pid_output = self.pid.calculate(0.0, off) * self.krlCoefficient
        new_ch1 = self.baseValue - pid_output
        new_ch2 = self.baseValue + pid_output
        new_ch3 = 100.0

        aMin, aMax = self.angleMinExtraSpeed, self.angleMaxExtraSpeed
        absOff = abs(off)
        if aMin > aMax:
            if absOff <= aMin:
                if absOff <= aMax:
                    extra = 100.0
                else:
                    rng = aMin - aMax
                    extra = 100.0 - ((absOff - aMax) * 100.0 / rng) if rng > 0 else 0.0
                extra *= self.kfbCoefficient
                new_ch1 += extra
                new_ch2 += extra
        else:
            if absOff < 90:
                extra = (100.0 - absOff * 100.0 / 90.0) * self.kfbCoefficient
                new_ch1 += extra
                new_ch2 += extra

        new_ch1 = max(self.minValue, min(self.maxValue, new_ch1))
        new_ch2 = max(self.minValue, min(self.maxValue, new_ch2))

        if self.reverse and self.useReverse == 1:
            new_ch1, new_ch2 = 100.0 - new_ch2, 100.0 - new_ch1

        if self.markerDetected and markerId >= 0 and self.getMarkerSide(markerId) == 0:
            new_ch1, new_ch2 = self.swapInvertChannels(new_ch1, new_ch2)

        self.ch1, self.ch2 = new_ch1, new_ch2
        if not self.ch3StartPending:
            self.ch3 = new_ch3
        self.sendToArduino()
