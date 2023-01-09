package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmJoint;
import org.firstinspires.ftc.teamcode.util.ArmReference;
import org.firstinspires.ftc.teamcode.util.RelativeTo;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.ArrayList;

public class ArmController {
    //TODO: Cleanup unused variables

    // Arm positions
    ArmPosition foldedPosition = new ArmPosition(-60, 160, -160, 135, UnitOfAngle.DEGREES, RelativeTo.ARM, "Folded Position");
    ArmPosition carryPosition = new ArmPosition(-30, 90, -90, 90, UnitOfAngle.DEGREES, RelativeTo.ARM, "Carry Position");
    ArmPosition closePosition = new ArmPosition(-30, 90, 90, -105, UnitOfAngle.DEGREES, RelativeTo.ARM, "Close Grab Position");
    ArmPosition farPosition = new ArmPosition(30, 15, 45, -90, UnitOfAngle.DEGREES, RelativeTo.ARM, "Far Grab Position");
    ArmPosition highPosition = new ArmPosition(-60, 30, -100, -25, UnitOfAngle.DEGREES, RelativeTo.ARM, "High Junction Position");
    ArmPosition middlePosition = new ArmPosition(-60, 30, 140, -65, UnitOfAngle.DEGREES, RelativeTo.ARM, "Middle Junction Position");
    ArmPosition lowPosition = new ArmPosition(-60, 90, 130, -115, UnitOfAngle.DEGREES, RelativeTo.ARM, "Low Junction Position");
    ArmPosition straightUpPosition = new ArmPosition(0, 0, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM, "Straight Up Position"); // 4 testing

    ArmPosition placeTransition = new ArmPosition(0, 0, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM, "Transition");
//    ArmPosition placeTransition = new ArmPosition(-20, 90, 90, -90, UnitOfAngle.DEGREES, RelativeTo.ARM, "Transition");
    ArmPosition carryTransition = placeTransition; //new ArmPosition(-45, 120, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM);
    ArmPosition grabTransition = placeTransition;
    ArmPosition zeroPosition = new ArmPosition(0, 0, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM, "Zero Position");

    ArmPosition targetPosition;
    ArmPosition lastPosition;

    ArmPosition endPosition;

    ArmPosition tickTarget;
    boolean currentSequenceDone = true;
    boolean autoHoming = false;
    int sequenceIndex = 0;

    // Gear Ratio = 188:1
    // Encoder Shaft = 28 pulses per revolution
    // Gearbox Output = 5281.1 pulses per revolution (*1.4 for small sprocket)
    public int grabberTicks = 0;
    public int turnTableTicks = 0;
    public int lowerTicks = 0;
    public int baseTicks = 0;
    public double grabberRotation = 0;
    public double grabberBend = 0;

    private final double manualAngleTolerance = 5; // Degrees
    private final double manualPositionTolerance = 5; // CM

    private Telemetry telemetry;

    // Arm sequences
    ArrayList<ArmPosition> foldedToCarry = new ArrayList<>();

    ArrayList<ArmPosition> upToCarry = new ArrayList<>();
    ArrayList<ArmPosition> carryToUp = new ArrayList<>();

    ArrayList<ArmPosition> carryToFold = new ArrayList<>();
    ArrayList<ArmPosition> carryToFar = new ArrayList<>();
    ArrayList<ArmPosition> carryToClose = new ArrayList<>();
    ArrayList<ArmPosition> carryToHigh = new ArrayList<>();
    ArrayList<ArmPosition> carryToMiddle = new ArrayList<>();
    ArrayList<ArmPosition> carryToLow = new ArrayList<>();

    ArrayList<ArmPosition> farToFolded = new ArrayList<>();
    ArrayList<ArmPosition> farToCarry = new ArrayList<>();
    ArrayList<ArmPosition> farToClose = new ArrayList<>();
    ArrayList<ArmPosition> farToHigh = new ArrayList<>();
    ArrayList<ArmPosition> farToMiddle = new ArrayList<>();
    ArrayList<ArmPosition> farToLow = new ArrayList<>();

    ArrayList<ArmPosition> closeToFolded = new ArrayList<>();
    ArrayList<ArmPosition> closeToCarry = new ArrayList<>();
    ArrayList<ArmPosition> closeToFar = new ArrayList<>();
    ArrayList<ArmPosition> closeToHigh = new ArrayList<>();
    ArrayList<ArmPosition> closeToMiddle = new ArrayList<>();
    ArrayList<ArmPosition> closeToLow = new ArrayList<>();

    ArrayList<ArmPosition> highToFold = new ArrayList<>();
    ArrayList<ArmPosition> highToCarry = new ArrayList<>();
    ArrayList<ArmPosition> highToMiddle = new ArrayList<>();
    ArrayList<ArmPosition> highToLow = new ArrayList<>();
    ArrayList<ArmPosition> highToFar = new ArrayList<>();
    ArrayList<ArmPosition> highToClose = new ArrayList<>();

    ArrayList<ArmPosition> middleToFolded = new ArrayList<>();
    ArrayList<ArmPosition> middleToCarry = new ArrayList<>();
    ArrayList<ArmPosition> middleToHigh = new ArrayList<>();
    ArrayList<ArmPosition> middleToLow = new ArrayList<>();
    ArrayList<ArmPosition> middleToFar = new ArrayList<>();
    ArrayList<ArmPosition> middleToClose = new ArrayList<>();

    ArrayList<ArmPosition> lowToFolded = new ArrayList<>();
    ArrayList<ArmPosition> lowToCarry = new ArrayList<>();
    ArrayList<ArmPosition> lowToHigh = new ArrayList<>();
    ArrayList<ArmPosition> lowToMiddle = new ArrayList<>();
    ArrayList<ArmPosition> lowToFar = new ArrayList<>();
    ArrayList<ArmPosition> lowToClose = new ArrayList<>();

    ArrayList<ArmPosition> currentSequence;
    ArrayList<ArmPosition> nextSequence;

    StringBuilder sb = new StringBuilder();

    public void initialize(Telemetry telemetry) {
        //TODO: Verify integrity of sequences

        this.telemetry = telemetry;
        // store the initial position of the arm
        currentSequenceDone = true;
        endPosition = foldedPosition;

        foldedToCarry.add(carryPosition);

        //--------------------------------- 4 testing
        carryToUp.add(straightUpPosition);
        upToCarry.add(carryPosition);

        //--------------------------------- Carry to other

        carryToFold.add(foldedPosition);
        // Initialize the current sequence to carryToFolded, and record that its done
        //currentSequence = carryToFold;

        //carryToFar.add(grabTransition);
        carryToFar.add(farPosition);

        //carryToClose.add(grabTransition);
        carryToClose.add(closePosition);

        //carryToHigh.add(placeTransition);
        carryToHigh.add(highPosition);

        //carryToMiddle.add(placeTransition);
        carryToMiddle.add(middlePosition);

        //carryToLow.add(placeTransition);
        carryToLow.add(lowPosition);

        //--------------------------------- Far to other

        //farToFolded.add(carryTransition);
        farToFolded.add(foldedPosition);

        //farToCarry.add(carryTransition);
        farToCarry.add(carryPosition);

        //farToClose.add(grabTransition);
        farToClose.add(closePosition);

        //farToHigh.add(placeTransition);
        farToHigh.add(highPosition);

        //farToMiddle.add(placeTransition);
        farToMiddle.add(middlePosition);

        //farToLow.add(placeTransition);
        farToLow.add(lowPosition);

        //--------------------------------- Close to other

        //closeToFolded.add(carryTransition);
        closeToFolded.add(foldedPosition);

        //closeToCarry.add(carryTransition);
        closeToCarry.add(carryPosition);

        //closeToFar.add(grabTransition);
        closeToFar.add(farPosition);

        //closeToHigh.add(placeTransition);
        closeToHigh.add(highPosition);

        //closeToMiddle.add(placeTransition);
        closeToMiddle.add(middlePosition);

        //closeToLow.add(placeTransition);
        closeToLow.add(lowPosition);

        //--------------------------------- High to other

        //highToFold.add(carryTransition);
        highToFold.add(foldedPosition);

        //highToCarry.add(carryTransition);
        highToCarry.add(carryPosition);

        //highToFar.add(grabTransition);
        highToFar.add(farPosition);

        //highToClose.add(grabTransition);
        highToClose.add(closePosition);

        //highToMiddle.add(placeTransition);
        highToMiddle.add(middlePosition);

        //highToLow.add(placeTransition);
        highToLow.add(lowPosition);

        //--------------------------------- Middle to other

        //middleToFolded.add(carryTransition);
        middleToFolded.add(foldedPosition);

        //middleToCarry.add(carryTransition);
        middleToCarry.add(carryPosition);

        //middleToFar.add(grabTransition);
        middleToFar.add(farPosition);

        //middleToClose.add(grabTransition);
        middleToClose.add(closePosition);

        //middleToHigh.add(placeTransition);
        middleToHigh.add(highPosition);

        //middleToLow.add(placeTransition);
        middleToLow.add(lowPosition);

        //--------------------------------- Low to other

        //lowToFolded.add(carryTransition);
        lowToFolded.add(foldedPosition);

        //lowToCarry.add(carryTransition);
        lowToCarry.add(carryPosition);

        //lowToFar.add(grabTransition);
        lowToFar.add(farPosition);

        //lowToClose.add(grabTransition);
        lowToClose.add(closePosition);

        //lowToHigh.add(placeTransition);
        lowToHigh.add(highPosition);

        //lowToMiddle.add(placeTransition);
        lowToMiddle.add(middlePosition);
    }

    public void updateArm(GamePadState gamePadState, Actuators actuators, Sensors sensors, boolean verbose) {
        // if the arm is homing: disable normal functions
        if (autoHoming) {
            autoHome(gamePadState, actuators, sensors, verbose);
        }
        else {
            //TODO: Replace goManual with semiAutonomous control
            //TODO: Replace trueManual with goManual
            //TODO: Revamp sequence system for semiAutonomous
            if (gamePadState.altMode) {
                trueManual(gamePadState, sensors);
            } else {
                godrickTheManual(gamePadState, sensors);
            }

            // Limit the angles to a certain range
            //TODO: Determine if we need a way to disable limits during use
            grabberBend = UtilityKit.limitToRange(grabberBend, -100.0, 100.0);
            grabberRotation = UtilityKit.limitToRange(grabberRotation, -100.0, 100.0);
            turnTableTicks = UtilityKit.limitToRange(turnTableTicks, sensors.turnData.getTicksLimit(ArmReference.PORT), sensors.turnData.getTicksLimit(ArmReference.STARBOARD));
            baseTicks = UtilityKit.limitToRange(turnTableTicks, sensors.baseData.getTicksLimit(ArmReference.STERN), sensors.baseData.getTicksLimit(ArmReference.BOW));
            lowerTicks = UtilityKit.limitToRange(turnTableTicks, sensors.lowerData.getTicksLimit(ArmReference.STERN), sensors.lowerData.getTicksLimit(ArmReference.BOW));
        }

        // check arm limits
        checkArmLimits(sensors);

        //TODO: Add variables that control velocity and power
        actuators.updateArm(sensors, grabberBend, grabberRotation, turnTableTicks, lowerTicks, baseTicks);
    }

    private void godrickTheManual(GamePadState gamePadState, Sensors sensors) {
//        d pad = left right - rotate the turn table
//        d pad = move grabber further away from robot
//        a, y = move grabber up/down
//        b = toggle gripper (In MotorController)
//        triggers = controls pitch (wrist bend joint 4)
//        bumpers = controls rotations (wrist, joint 3)

        //TODO: Add limits for x/y movement

        ArmJoint lowerData = sensors.lowerData;
        ArmJoint baseData = sensors.baseData;
        ArmJoint turnData = sensors.turnData;

        Vector2D target = getTargetPosition(sensors);
        Vector2D current = sensors.grabberPosition;

        boolean lb = gamePadState.leftBumper;
        boolean rb = gamePadState.rightBumper;
        double lt = gamePadState.leftTrigger;
        double rt = gamePadState.rightTrigger;

        double addX = 0; // CM
        double addY = 0; // CM

        double turnTableAngle = 0;
        double addWristRotation = 0;
        double addWristBend = 0;

        if (rb && !lb) {
            addWristRotation = .5;
        } else if (lb && !rb) {
            addWristRotation = -.5;
        }

        if (rt != 0 && lt == 0) {
            addWristBend = rt * .5;
        } else if (lt != 0 && rt == 0) {
            addWristBend = lt * -.5;
        }

        if (gamePadState.a && !gamePadState.y) {
            if (current.getY() < target.getY()+manualPositionTolerance) {
                addY = -2.5;
            }
        } else if ( gamePadState.y && !gamePadState.a) {
            if (current.getY() > target.getY()-manualPositionTolerance) {
                addY = 2.5;
            }
        }

        if (gamePadState.dPadUp) {
            if (current.getX() < target.getX()+manualPositionTolerance) {
                addX = 2.5;
            }
        } else if (gamePadState.dPadDown) {
            if (current.getX() > target.getX()-manualPositionTolerance) {
                addX = -2.5;
            }
        }

        if (gamePadState.dPadRight) {
            if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(turnTableTicks)-manualAngleTolerance) {
                turnTableAngle = 2.5;
            }
        } else if (gamePadState.dPadLeft) {
            if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(turnTableTicks)+manualAngleTolerance) {
                turnTableAngle = -2.5;
            }
        }

        findAngles(target.getX()+addX, target.getY()+addY, sensors);
        turnTableTicks += (int) (turnTableAngle * UtilityKit.ticksPerDegreeAtJoint);
        grabberBend += addWristBend;
        grabberRotation += addWristRotation;
    }

    private void trueManual(GamePadState gamePadState, Sensors sensors) {
//        d pad = left right - rotate the turn table
//        d pad = up down - controls the shoulder (joint 1)
//        a, y = controls the elbow (joint 2)
//        b = toggle gripper (In MotorController)
//        triggers = controls pitch (wrist bend joint 4)
//        bumpers = controls rotations (wrist, joint 3)

        ArmJoint lowerData = sensors.lowerData;
        ArmJoint baseData = sensors.baseData;
        ArmJoint turnData = sensors.turnData;

        boolean lb = gamePadState.leftBumper;
        boolean rb = gamePadState.rightBumper;
        double lt = gamePadState.leftTrigger;
        double rt = gamePadState.rightTrigger;

        double addBaseAngle = 0;
        double addLowerAngle = 0;
        double turnTableAngle = 0;
        double addWristRotation = 0;
        double addWristBend = 0;

        if (rb && !lb) {
            addWristRotation = .5;
        } else if (lb && !rb) {
            addWristRotation = -.5;
        }

        if (rt != 0 && lt == 0) {
            addWristBend = rt * .5;
        } else if (lt != 0 && rt == 0) {
            addWristBend = lt * -.5;
        }

        if (gamePadState.a && !gamePadState.y) {
            if (lowerData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(lowerTicks)+manualAngleTolerance) {
                addLowerAngle = -2.5;
            }
        } else if ( gamePadState.y && !gamePadState.a) {
            if (lowerData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(lowerTicks)-manualAngleTolerance) {
                addLowerAngle = 2.5;
            }
        }

        if (gamePadState.dPadUp) {
            if (baseData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(baseTicks)-manualAngleTolerance) {
                addBaseAngle = 2.5;
            }
        } else if (gamePadState.dPadDown) {
            if (baseData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(baseTicks)+manualAngleTolerance) {
                addBaseAngle = -2.5;
            }
        }

        if (gamePadState.dPadRight) {
            if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(turnTableTicks)-manualAngleTolerance) {
                turnTableAngle = 2.5;
            }
        } else if (gamePadState.dPadLeft) {
            if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(turnTableTicks)+manualAngleTolerance) {
                turnTableAngle = -2.5;
            }
        }

        turnTableTicks += (int) (turnTableAngle * UtilityKit.ticksPerDegreeAtJoint);
        lowerTicks += (int) (addLowerAngle * UtilityKit.ticksPerDegreeAtJoint);
        baseTicks += (int) (addBaseAngle * UtilityKit.ticksPerDegreeAtJoint);
        grabberBend += addWristBend;
        grabberRotation += addWristRotation;
    }

    private void autoHome(GamePadState gamePadState, Actuators actuators, Sensors sensors, boolean verbose) {
        //TODO: ensure latter joints are out of the way during homing process
        //TODO: create sequence for homing each joint
    }

    private void checkArmLimits(Sensors sensors){
        //TODO: Create system where when the limit switch is activated the arm will move away from the limit switch
        //TODO: Determine whether or not we should reset position when a switch is activated outside of autoHome
    }

    private void findAngles(double x, double y, Sensors sensors) {
        double distance = Math.sqrt(x*x+y*y);
        double length1 = sensors.baseData.getPositionDistance(UnitOfDistance.CM);
        double length2 = sensors.lowerData.getPositionDistance(UnitOfDistance.CM);
        double extraAngle = UtilityKit.atan(y/x, UnitOfAngle.DEGREES);

        double baseAngle = 90 - extraAngle - UtilityKit.acos((length1*length1+distance*distance-length2*length2)/(2*length1*distance), UnitOfAngle.DEGREES);
        double lowerAngle = 180 - UtilityKit.acos((length1*length1+length2*length2-distance*distance)/(2*length1*length2), UnitOfAngle.DEGREES);

        baseTicks = UtilityKit.armDegreesToTicks(baseAngle);
        lowerTicks = UtilityKit.armDegreesToTicks(lowerAngle);
    }

    private Vector2D getTargetPosition(Sensors sensors) {
        double x = sensors.baseData.getPositionDistance(UnitOfDistance.CM)*UtilityKit.sin(UtilityKit.armTicksToDegrees(baseTicks), UnitOfAngle.DEGREES) + sensors.lowerData.getPositionDistance(UnitOfDistance.CM)*UtilityKit.sin(UtilityKit.armTicksToDegrees(lowerTicks), UnitOfAngle.DEGREES);
        double y = sensors.baseData.getPositionDistance(UnitOfDistance.CM)*UtilityKit.cos(UtilityKit.armTicksToDegrees(baseTicks), UnitOfAngle.DEGREES) + sensors.lowerData.getPositionDistance(UnitOfDistance.CM)*UtilityKit.cos(UtilityKit.armTicksToDegrees(lowerTicks), UnitOfAngle.DEGREES);
        return new Vector2D(x, y);
    }
}
