package org.firstinspires.ftc.teamcode.arm;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.Actuators;
import org.firstinspires.ftc.teamcode.system.GamePadState;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.ArrayList;

public class ArmController {
    //TODO: Cleanup unused variables

    boolean currentSequenceDone = true;
    boolean autoHoming = false;
    boolean baseHome = true;
    boolean lowerHome = true;
    int sequenceIndex = 0;

    // Gear Ratio = 188:1
    // Encoder Shaft = 28 pulses per revolution
    // Gearbox Output = 5281.1 pulses per revolution (*1.4 for small sprocket)
    public int turnTableTicks = 0;
    public int lowerTicks = 0;
    public int baseTicks = 0;
    public double grabberRotation = 0;
    public double grabberBend = 0;

    private final double manualAngleTolerance = 5; // Degrees
    private final double manualPositionTolerance = 5; // CM

    public ArmJoint table;
    public ArmJoint base;
    public ArmJoint lower;

    private Telemetry telemetry;


    ArrayList<ArmPosition> currentSequence;
    ArrayList<ArmPosition> nextSequence;

    StringBuilder sb = new StringBuilder();
    MotionSequenceDirector motionSequenceDirector;

    public void initialize(Sensors sensors, Telemetry telemetry) {
        table = new ArmJoint("TurntableJoint", UnitOfAngle.DEGREES, 180, ArmReference.PORT, 180, ArmReference.STARBOARD, 20, 15, UnitOfDistance.CM);
        base = new ArmJoint("BaseJointA", UnitOfAngle.DEGREES, 90, ArmReference.BOW, 80, ArmReference.STERN, 2.4, 28.8, UnitOfDistance.CM);
        lower = new ArmJoint("LowerJoint", UnitOfAngle.DEGREES, 170, ArmReference.BOW, 0, ArmReference.STERN, -2.4, 28.8, UnitOfDistance.CM);

        motionSequenceDirector = new MotionSequenceDirector(sensors, telemetry);

        table = sensors.turnData;
        base = sensors.baseData;
        lower = sensors.lowerData;

        baseHome = false;
        lowerHome = false;
        autoHoming = true;

        this.telemetry = telemetry;
        // store the initial position of the arm
        currentSequenceDone = true;
    }

    public void updateArm(GamePadState gamePadState, Actuators actuators, Sensors sensors, boolean verbose) {
        // if the arm is homing: disable normal functions

        if (autoHoming) {
            autoHome(sensors, actuators, verbose);
        }
        else {
            godrickAttackMode(gamePadState, sensors);

            //TODO: Replace goManual with semiAutonomous control
            //TODO: Replace trueManual with goManual
            //TODO: Revamp sequence system for semiAutonomous
            //if (gamePadState.altMode) {
                //trueManual(gamePadState);
           // } else {
                //godrickTheManual(gamePadState, sensors);
           // }

            // Limit the angles to a certain range
            //TODO: Determine if we need a way to disable limits during use
            //grabberBend = UtilityKit.limitToRange(grabberBend, -100.0, 100.0);
            //grabberRotation = UtilityKit.limitToRange(grabberRotation, -100.0, 100.0);
            //turnTableTicks = UtilityKit.limitToRange(turnTableTicks, sensors.turnData.getTicksLimit(ArmReference.PORT), sensors.turnData.getTicksLimit(ArmReference.STARBOARD));
            //baseTicks = UtilityKit.limitToRange(turnTableTicks, sensors.baseData.getTicksLimit(ArmReference.STERN), sensors.baseData.getTicksLimit(ArmReference.BOW));
            //lowerTicks = UtilityKit.limitToRange(turnTableTicks, sensors.lowerData.getTicksLimit(ArmReference.STERN), sensors.lowerData.getTicksLimit(ArmReference.BOW));
        }

        // check arm limits
        checkArmLimits(sensors);

        if (verbose) {
            telemetry.addData("table target: ", turnTableTicks);
            telemetry.addData("base target: ", baseTicks);
            telemetry.addData("lower target: ", lowerTicks);
            telemetry.addData("rotate target: ", grabberRotation);
            telemetry.addData("pitch target: ", grabberBend);
        }
    }

    // Manual input for inverse kinematics
    private void godrickTheManual(GamePadState gamePadState, Sensors sensors) {
//        d pad = left right - rotate the turn table
//        d pad = move grabber further away from robot
//        a, y = move grabber up/down
//        b = toggle gripper (In MotorController)
//        triggers = controls pitch (wrist bend joint 4)
//        bumpers = controls rotations (wrist, joint 3)

        //TODO: Add limits for x/y movement

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
            if (table.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(turnTableTicks)-manualAngleTolerance) {
                turnTableAngle = 2.5;
            }
        } else if (gamePadState.dPadLeft) {
            if (table.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(turnTableTicks)+manualAngleTolerance) {
                turnTableAngle = -2.5;
            }
        }

        findAngles(target.getX()+addX, target.getY()+addY);
        turnTableTicks += (int) (turnTableAngle * UtilityKit.ticksPerDegreeAtJoint);
        grabberBend += addWristBend;
        grabberRotation += addWristRotation;
    }

    private void trueManual(GamePadState gamePadState) {
//        d pad = left right - rotate the turn table
//        d pad = up down - controls the shoulder (joint 1)
//        a, y = controls the elbow (joint 2)
//        b = toggle gripper (In MotorController)
//        triggers = controls pitch (wrist bend joint 4)
//        bumpers = controls rotations (wrist, joint 3)

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
            //if (lowerData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(lowerTicks)+manualAngleTolerance) {
                addLowerAngle = -2.5;
            //}
        } else if ( gamePadState.y && !gamePadState.a) {
            //if (lowerData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(lowerTicks)-manualAngleTolerance) {
                addLowerAngle = 2.5;
           // }
        }

        if (gamePadState.dPadUp) {
            //if (baseData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(baseTicks)-manualAngleTolerance) {
                addBaseAngle = 2.5;
            //}
        } else if (gamePadState.dPadDown) {
            //if (baseData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(baseTicks)+manualAngleTolerance) {
                addBaseAngle = -2.5;
            //}
        }

        if (gamePadState.dPadRight) {
            //if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(turnTableTicks)-manualAngleTolerance) {
                turnTableAngle = 2.5;
            //}
        } else if (gamePadState.dPadLeft) {
            //if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(turnTableTicks)+manualAngleTolerance) {
                turnTableAngle = -2.5;
            //}
        }

        turnTableTicks += (int) (turnTableAngle * UtilityKit.ticksPerDegreeAtJoint);
        lowerTicks += (int) (addLowerAngle * UtilityKit.ticksPerDegreeAtJoint);
        baseTicks += (int) (addBaseAngle * UtilityKit.ticksPerDegreeAtJoint);
        grabberBend += addWristBend;
        grabberRotation += addWristRotation;
    }

    // TODO: coach has checked this and approves it for use with the note below for improvement
    private void autoHome(Sensors sensors, Actuators actuators, boolean verbose) {
        if (sensors.base) {
            baseHome = true;
            actuators.baseSegment.reset();
            actuators.baseSegment2.reset();
            baseTicks = 0;
            base.resetPosition(ArmReference.STERN);
        }
        if (sensors.lowerA) {
            lowerHome = true;
            actuators.lowerSegment.reset();
            lowerTicks = 0;
            lower.resetPosition(ArmReference.BOW);
        }

        if (baseHome && lowerHome) {
            autoHoming = false;
        }
        else { // slowly move arm until it presses the button
            if (!baseHome) {
                baseTicks--; // TODO: the problem with this kind of control is that the arm may not be keeping up
            }
            else {
                lowerTicks++; // TODO: the problem with this kind of control is that the arm may not be keeping up
            }
        }

        if (verbose) {
            telemetry.addData("Homing ", autoHoming);
        }
    }

    // TODO: FOR EXAMPLE, in autoMode when the attack button is pressed, we look for a cone and grab it
    // and when the B button is pressed, it looks for a pole and places it
    private void godrickAttackMode(GamePadState gamePadState, Sensors sensors){

        if(gamePadState.a){
           // Log.i("ArmController", "A Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.CarryToGrabToCarry);

        }
        else if(gamePadState.b){
            motionSequenceDirector.requestNewSequence(MotionSequenceName.CarryToPlaceToCarry);
        }
        else if(gamePadState.x){
            motionSequenceDirector.requestNewSequence(MotionSequenceName.CarryToHome);
        }
        else if(gamePadState.y){
            motionSequenceDirector.requestNewSequence(MotionSequenceName.HomeToCarry);
        }
        else{
            //Log.i("ArmController", "Nothing detected in attack mode");
        }
    }

    private void checkArmLimits(Sensors sensors){
        if (sensors.base) {
            baseTicks++;
        }

        if (sensors.lowerA) {
            lowerTicks++;
        }

        else if (sensors.lowerB) {
            lowerTicks--;
        }
    }

    private void findAngles(double x, double y) {
        double distance = Math.sqrt(x*x+y*y);
        double length1 = base.getPositionDistance(UnitOfDistance.CM);
        double length2 = lower.getPositionDistance(UnitOfDistance.CM);
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

    double startDistance;
    Vector2D nextPosition;

}
