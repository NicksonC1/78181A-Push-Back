#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vrc import *
from vexcode_vrc.events import get_Task_func

# Brain should be defined by default
brain = Brain()

conveyorMotor = Motor("ConveyorMotor", 3)
intakeMotor = Motor("IntakeMotor", 4)
bumper = Bumper("Bumper", 5)
aiVision = AiVision("aiVision", 6)
gps = GPS("GPS", 7)
optical = Optical("Optical", 8)
distance = Distance("Distance", 9)
drivetrain = Drivetrain("drivetrain", 0)

# 'Enum' class for AI Vision GameElements
class GameElements:
    BLUE_BLOCK = 0
    RED_BLOCK = 1

# Color to String Helper
def convertColorToString(col):
    if col == RED:
        return "red"
    if col == GREEN:
        return "green"
    if col == BLUE:
        return "blue"
    if col == WHITE:
        return "white"
    if col == YELLOW:
        return "yellow"
    if col == ORANGE:
        return "orange"
    if col == PURPLE:
        return "purple"
    if col == CYAN:
        return "cyan"
    if col == RED_VIOLET:
        return "red_violet"
    if col == VIOLET:
        return "violet"
    if col == BLUE_VIOLET:
        return "blue_violet"
    if col == BLUE_GREEN:
        return "blue_green"
    if col == YELLOW_GREEN:
        return "yellow_green"
    if col == YELLOW_ORANGE:
        return "yellow_orange"
    if col == RED_ORANGE:
        return "red_orange"
    if col == BLACK:
        return "black"
    if col == TRANSPARENT:
        return "transparent"
    return ""

#endregion VEXcode Generated Robot Configuration

loopDelayMs = 10


class IntakeMode:
    off = "off"
    intake = "intake"
    score = "score"
    spit = "spit"
    mid = "mid"


def clamp(val, low, high):
    return max(low, min(high, val))


class Intake:
    def __init__(self, intakeMot, convMot):
        self.intakeMot = intakeMot
        self.convMot = convMot
        self.curMode = IntakeMode.off

    def _spin(self, mot, spdPct):
        spdPct = clamp(spdPct, -100, 100)
        if abs(spdPct) < 1:
            mot.stop()
            return

        mot.set_velocity(abs(spdPct), PERCENT)
        mot.spin(FORWARD if spdPct > 0 else REVERSE)

    def setMode(self, mode):
        self.curMode = mode

        if mode == IntakeMode.intake:
            intakeSpd = 100
            convSpd = 100
        elif mode == IntakeMode.mid:
            intakeSpd = 100
            convSpd = 60
        elif mode == IntakeMode.score:
            intakeSpd = 100
            convSpd = 100
        elif mode == IntakeMode.spit:
            intakeSpd = -100
            convSpd = -100
        else:
            intakeSpd = 0
            convSpd = 0

        self._spin(self.intakeMot, intakeSpd)
        self._spin(self.convMot, convSpd)

    def stop(self):
        self.setMode(IntakeMode.off)

    def pulseSpit(self, durMs=140, spdPct=85):
        self._spin(self.intakeMot, -abs(spdPct))
        self._spin(self.convMot, -abs(spdPct))
        wait(durMs, MSEC)

        if self.curMode == IntakeMode.spit:
            self.stop()
        else:
            self.setMode(self.curMode)

    def intakeTillSeen(self, timeoutMs=2200):
        self.setMode(IntakeMode.intake)

        elapsed = 0
        while elapsed < timeoutMs:
            if optical.is_near_object() or distance.is_object_detected():
                return True
            wait(loopDelayMs, MSEC)
            elapsed += loopDelayMs
        return False


def moveToPointCore(x, y, fwd=True, spdPct=100):
    dx = x - gps.x_position(MM)
    dy = y - gps.y_position(MM)
    d = math.hypot(dx, dy)
    a = math.degrees(math.atan2(dy, dx))

    drivetrain.set_drive_velocity(clamp(spdPct, 10, 100), PERCENT)

    if not fwd:
        drivetrain.turn_to_heading(270 - a, DEGREES)
        drivetrain.drive_for(REVERSE, d, MM)
    else:
        drivetrain.turn_to_heading(90 - a, DEGREES)
        drivetrain.drive_for(FORWARD, d, MM)


class Chassis:
    def __init__(self, dt, gpsSense):
        self.dt = dt
        self.gps = gpsSense
        self.distTravel = -1
        self.moveRun = False

    def _headingToPoint(self, x, y, forward=True):
        dx = x - self.gps.x_position(MM)
        dy = y - self.gps.y_position(MM)
        targetAng = math.degrees(math.atan2(dy, dx))
        heading = (90 - targetAng) if forward else (270 - targetAng)
        return heading % 360

    def turnToPoint(self, x, y, forward=True):
        self.dt.turn_to_heading(self._headingToPoint(x, y, forward), DEGREES)

    def _moveToPointSync(self, x, y, fwd=True, spdPct=100):
        self.distTravel = 0
        self.moveRun = True
        moveToPointCore(x, y, fwd=fwd, spdPct=spdPct)
        self.dt.stop()
        self.distTravel = -1
        self.moveRun = False

    def moveToPoint(self, x, y, fwd=True, spdPct=100):
        # Synchronous-only path to avoid any async/thread race issues.
        self._moveToPointSync(x, y, fwd, spdPct)

    def waitUntil(self, distMm):
        wait(10, MSEC)
        while self.distTravel <= distMm and self.distTravel != -1:
            wait(10, MSEC)

    def waitUntilDone(self):
        wait(10, MSEC)
        while self.moveRun or self.distTravel != -1:
            wait(10, MSEC)

    def chain(self, pts, fwd=True, spdPct=100):
        for x, y in pts:
            self.moveToPoint(x, y, fwd=fwd, spdPct=spdPct)

    def driveForRel(self, distMm, spdPct=100):
        headingRad = math.radians(self.gps.heading())
        startX = self.gps.x_position(MM)
        startY = self.gps.y_position(MM)

        targetX = startX + math.sin(headingRad) * distMm
        targetY = startY + math.cos(headingRad) * distMm

        self.moveToPoint(targetX, targetY, fwd=(distMm >= 0), spdPct=spdPct)


chassis = Chassis(drivetrain, gps)
intake = Intake(intakeMotor, conveyorMotor)


def showDebug():
    brain.screen.clear_screen()
    brain.screen.print("x: " + str(round(gps.x_position(MM), 1)))
    brain.screen.next_row()
    brain.screen.print("y: " + str(round(gps.y_position(MM), 1)))
    brain.screen.next_row()
    brain.screen.print("hdg: " + str(round(gps.heading(), 1)))


def intakeT():
    return intake.intakeTillSeen(2200)


def openRoute():
    drivetrain.set_heading(55, DEGREES)

    intake.setMode(IntakeMode.intake)
    chassis.moveToPoint(-600, 600, fwd=True, spdPct=100)
    chassis.waitUntil(220)
    intake.intakeTillSeen(1800)
    chassis.waitUntilDone()

    chassis.moveToPoint(-1200, 1200, fwd=False, spdPct=95)


def sweepRoute():
    intake.setMode(IntakeMode.intake)
    pts = [(-1000, 900), (-750, 700), (-500, 520)]
    chassis.chain(pts, fwd=True, spdPct=85)


def scoreRoute():
    drivetrain.turn_to_heading(270, DEGREES)

    intake.setMode(IntakeMode.score)
    wait(550, MSEC)
    intake.setMode(IntakeMode.intake)


def parkRoute():
    chassis.driveForRel(-360, spdPct=90)

    drivetrain.set_drive_velocity(50, PERCENT)
    drivetrain.drive(FORWARD)

    elapsed = 0
    while elapsed < 1500:
        hue = optical.hue()
        if optical.is_near_object() and 0 < hue < 12:
            break
        wait(loopDelayMs, MSEC)
        elapsed += loopDelayMs

    drivetrain.stop()


def main():
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    intakeMotor.set_velocity(100, PERCENT)
    conveyorMotor.set_velocity(100, PERCENT)

    drivetrain.set_heading(55, DEGREES)
    intake.setMode(IntakeMode.intake)

    # Use relative target so it always moves regardless of spawn position.
    startX = gps.x_position(MM)
    startY = gps.y_position(MM)
    targetX = startX - 600
    targetY = startY + 600

    brain.screen.clear_screen()
    brain.screen.print("start: " + str(round(startX, 1)) + ", " + str(round(startY, 1)))
    brain.screen.next_row()
    brain.screen.print("target: " + str(round(targetX, 1)) + ", " + str(round(targetY, 1)))

    chassis.moveToPoint(targetX, targetY, spdPct=100)

    # openRoute()
    # sweepRoute()
    # scoreRoute()
    # parkRoute()

    # intake.stop()
    # drivetrain.stop()
    # showDebug()


# VR threads TEST -- Do not delete
vr_thread(main)
