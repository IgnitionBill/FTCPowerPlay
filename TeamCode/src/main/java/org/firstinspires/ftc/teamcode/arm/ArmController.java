package org.firstinspires.ftc.teamcode.arm;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sequence.MotionSequenceDirector;
import org.firstinspires.ftc.teamcode.sequence.MotionSequenceName;
import org.firstinspires.ftc.teamcode.system.Actuators;
import org.firstinspires.ftc.teamcode.system.GamePadState;
import org.firstinspires.ftc.teamcode.system.Godrick;
import org.firstinspires.ftc.teamcode.system.Sensors;

/**
 * ArmController is the top level of controller for the arm.
 * During the initialize function, it stores references and creates things.
 * During the update function it checks which ArmControlMode is active and switches to that function.
 *
 * AutoHome is always the first ArmControlMode activated, to verify the arm position.
 * Manual mode is used to control individual motors for testing purposes.
 * AttackMode automates pick and place actions.
 */
public class ArmController {
    //TODO: Cleanup unused variables

    ArmControlMode armControlMode = ArmControlMode.AUTO_HOME; // always default to AUTO_HOME

    boolean currentSequenceDone = true;
    boolean baseHome = true;
    boolean lowerHome = true;

    // Gear Ratio = 188:1
    // Encoder Shaft = 28 pulses per revolution
    // Gearbox Output = 5281.1 pulses per revolution (*1.4 for small sprocket)
//    public int turnTableTicks = 0; // th0
//    public int baseTicks = 0; // th1
//    public int lowerTicks = 0; // th2

    private final double manualAngleTolerance = 5; // Degrees
    private final double manualPositionTolerance = 5; // CM

    private Telemetry telemetry;

    MotionSequenceDirector motionSequenceDirector;
    Godrick godrick;
    Sensors sensors;
    Actuators actuators;
    GamePadState gamePadState;

    public void initialize(Godrick godrickInput) {
        godrick = godrickInput;
        sensors = godrick.sensors;
        actuators = godrick.actuators;
        gamePadState = godrick.gamePadState;
        telemetry = godrick.telemetry;
        motionSequenceDirector = godrick.motionSequenceDirector;

        baseHome = false;
        lowerHome = false;

        // store the initial position of the arm
        currentSequenceDone = true;

        armControlMode = ArmControlMode.AUTO_HOME;
    }

    /**
     * The update arm function
     * @param verbose
     */
    public void updateArm(boolean verbose) {
        // if the arm is homing: disable normal functions
        if (armControlMode == ArmControlMode.AUTO_HOME) {
            autoHome(sensors, actuators, true);
        }
        else if (armControlMode == ArmControlMode.ATTACK) {
            godrickAttackMode(gamePadState, sensors);
        }
        else if(armControlMode == ArmControlMode.MANUAL){
            godrickTheManual();
        }
        else if(armControlMode == ArmControlMode.EASY){
            godrickTakesItEasy();
        }
        else if(armControlMode == ArmControlMode.DEMO){
            godrickDemo();
        }

        // check arm limits
        checkArmLimits(sensors);

//        if (verbose) {
//            telemetry.addData("table target: ", turnTableTicks);
//            telemetry.addData("base target: ", baseTicks);
//            telemetry.addData("lower target: ", lowerTicks);
//            telemetry.addData("rotate target: ", grabberRoll);
//            telemetry.addData("pitch target: ", grabberPitch);
//        }
    }

    private void godrickTheManual(){
        if(gamePadState.dPadLeft){
            // move the turn table counter-clockwise
            godrick.arm.turntable.incrementTargetAngle(1.0);
        }
        if(gamePadState.dPadRight){
            // move the turntable clockwise
            godrick.arm.turntable.incrementTargetAngle(-1.0);
        }
        if(gamePadState.dPadUp){
            // move the arm forward
            godrick.arm.baseJoint.incrementTargetAngle(1.0);
        }
        if(gamePadState.dPadDown){
            // move the arm back
            godrick.arm.baseJoint.incrementTargetAngle(-1.0);
        }
        if(gamePadState.y){
            // move seg 2 forward
            godrick.arm.elbowJoint.incrementTargetAngle(-1.0);
        }
        if(gamePadState.a){
            // move seg 2 back
            godrick.arm.elbowJoint.incrementTargetAngle(1.0);
        }
    }

    private void godrickTakesItEasy(){

        if(gamePadState.dPadUp){
            // move the arm gripper forward via inverse kinematics
            // x += diff;

        }
        if(gamePadState.dPadDown){
            // move the arm gripper backward via inverse kinematics
            // x -= diff;
        }
        if(gamePadState.y){
            // move the arm gripper upward via inverse kinematics
            // y += diff;
        }
        if(gamePadState.a){
            // move the arm gripper downward via inverse kinematics
            // y -= diff;
        }
        if (gamePadState.dPadLeft) {
            // Rotate counter clockwise
        }
        if (gamePadState.dPadRight) {
            // Rotate clockwise
        }
    }

    // Manual input for inverse kinematics
//    private void godrickTheManual() {
////        d pad = left right - rotate the turn table
////        d pad = move grabber further away from robot
////        a, y = move grabber up/down
////        b = toggle gripper (In MotorController)
////        triggers = controls pitch (wrist bend joint 4)
////        bumpers = controls rotations (wrist, joint 3)
//
//        Vector2D target = getTargetPosition(sensors);
//        //Vector2D current = sensors.grabberPosition;
//
//        boolean lb = gamePadState.leftBumper;
//        boolean rb = gamePadState.rightBumper;
//        double lt = gamePadState.leftTrigger;
//        double rt = gamePadState.rightTrigger;
//
//        double addX = 0; // CM
//        double addY = 0; // CM
//
//        double turnTableAngle = 0;
//        double addWristRotation = 0;
//        double addWristBend = 0;
//
//        if (rb && !lb) {
//            addWristRotation = .5;
//        } else if (lb && !rb) {
//            addWristRotation = -.5;
//        }
//
//        if (rt != 0 && lt == 0) {
//            addWristBend = rt * .5;
//        } else if (lt != 0 && rt == 0) {
//            addWristBend = lt * -.5;
//        }
//
////        if (gamePadState.a && !gamePadState.y) {
////            if (current.getY() < target.getY()+manualPositionTolerance) {
////                addY = -2.5;
////            }
////        } else if ( gamePadState.y && !gamePadState.a) {
////            if (current.getY() > target.getY()-manualPositionTolerance) {
////                addY = 2.5;
////            }
////        }
////
////        if (gamePadState.dPadUp) {
////            if (current.getX() < target.getX()+manualPositionTolerance) {
////                addX = 2.5;
////            }
////        } else if (gamePadState.dPadDown) {
////            if (current.getX() > target.getX()-manualPositionTolerance) {
////                addX = -2.5;
////            }
////        }
///*
//        if (gamePadState.dPadRight) {
//            if (table.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(turnTableTicks)-manualAngleTolerance) {
//                turnTableAngle = 2.5;
//            }
//        } else if (gamePadState.dPadLeft) {
//            if (table.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(turnTableTicks)+manualAngleTolerance) {
//                turnTableAngle = -2.5;
//            }
//        }
//*/ // TODO:Fix this ARM JOINT THING
//        findAngles(target.getX()+addX, target.getY()+addY);
//        turnTableTicks += (int) (turnTableAngle * UtilityKit.ticksPerDegreeAtJoint);
//        grabberPitch += addWristBend;
//        grabberRoll += addWristRotation;
//    }
//
//    private void trueManual(GamePadState gamePadState) {
////        d pad = left right - rotate the turn table
////        d pad = up down - controls the shoulder (joint 1)
////        a, y = controls the elbow (joint 2)
////        b = toggle gripper (In MotorController)
////        triggers = controls pitch (wrist bend joint 4)
////        bumpers = controls rotations (wrist, joint 3)
//
//        boolean lb = gamePadState.leftBumper;
//        boolean rb = gamePadState.rightBumper;
//        double lt = gamePadState.leftTrigger;
//        double rt = gamePadState.rightTrigger;
//
//        double addBaseAngle = 0;
//        double addLowerAngle = 0;
//        double turnTableAngle = 0;
//        double addWristRotation = 0;
//        double addWristBend = 0;
//
//        if (rb && !lb) {
//            addWristRotation = .5;
//        } else if (lb && !rb) {
//            addWristRotation = -.5;
//        }
//
//        if (rt != 0 && lt == 0) {
//            addWristBend = rt * .5;
//        } else if (lt != 0 && rt == 0) {
//            addWristBend = lt * -.5;
//        }
//
//        if (gamePadState.a && !gamePadState.y) {
//            //if (lowerData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(lowerTicks)+manualAngleTolerance) {
//                addLowerAngle = -2.5;
//            //}
//        } else if ( gamePadState.y && !gamePadState.a) {
//            //if (lowerData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(lowerTicks)-manualAngleTolerance) {
//                addLowerAngle = 2.5;
//           // }
//        }
//
//        if (gamePadState.dPadUp) {
//            //if (baseData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(baseTicks)-manualAngleTolerance) {
//                addBaseAngle = 2.5;
//            //}
//        } else if (gamePadState.dPadDown) {
//            //if (baseData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(baseTicks)+manualAngleTolerance) {
//                addBaseAngle = -2.5;
//            //}
//        }
//
//        if (gamePadState.dPadRight) {
//            //if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) > UtilityKit.armTicksToDegrees(turnTableTicks)-manualAngleTolerance) {
//                turnTableAngle = 2.5;
//            //}
//        } else if (gamePadState.dPadLeft) {
//            //if (turnData.getCurrentAngle(UnitOfAngle.DEGREES) < UtilityKit.armTicksToDegrees(turnTableTicks)+manualAngleTolerance) {
//                turnTableAngle = -2.5;
//            //}
//        }
//
//        turnTableTicks += (int) (turnTableAngle * UtilityKit.ticksPerDegreeAtJoint);
//        lowerTicks += (int) (addLowerAngle * UtilityKit.ticksPerDegreeAtJoint);
//        baseTicks += (int) (addBaseAngle * UtilityKit.ticksPerDegreeAtJoint);
//        grabberPitch += addWristBend;
//        grabberRoll += addWristRotation;
//    }

    // TODO: add autohome for turntable
    private void autoHome(Sensors sensors, Actuators actuators, boolean verbose) {
        Log.e("ArmController", "Running autoHome");
        // if the base button is pressed, note that the joint is home and reset the motor encoders
        if (sensors.baseLimit) {
            baseHome = true;
            actuators.baseSegment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            actuators.baseSegment2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //godrick.arm.baseJointA.setTargetAngle(0);
        }
        // if the lower button is pressed, note that it is home and reset the motor encoders
        if (sensors.lowerLimitB) {
            lowerHome = true;
            actuators.lowerSegment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //godrick.arm.elbowJoint.setTargetAngle(0);
        }
        // after both buttons have been pressed, autohome complete, next mode
        if (baseHome && lowerHome) {
            armControlMode = ArmControlMode.MANUAL;
            Log.e("ArmController", "autoHome complete");
        }
        // until both buttons have been pressed,
        else { // slowly move each arm toward the button
            if (!baseHome) {
                godrick.arm.baseJoint.setTargetAngle(godrick.arm.baseJoint.getAngleDeg() - 1.0);
            }
            if(!lowerHome) {
                godrick.arm.elbowJoint.setTargetAngle(godrick.arm.elbowJoint.getAngleDeg() + 1.0);
            }
        }

        if (verbose) {
            telemetry.addData("ArmControlMode ", armControlMode.toString());
        }
    }

    private void godrickDemo(){
        motionSequenceDirector.update();
    }

    // TODO: FOR EXAMPLE, in autoMode when the attack button is pressed, we look for a cone and grab it
    // and when the B button is pressed, it looks for a pole and places it
    private void godrickAttackMode(GamePadState gamePadState, Sensors sensors){
        // update the motion director
        motionSequenceDirector.update();

        if(gamePadState.a){
            Log.e("ArmController", "Grab Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.CarryToGrabToCarry);
        }
        else if(gamePadState.b){
            Log.e("ArmController", "Place Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.CarryToPlaceToCarry);
        }
        else if(gamePadState.x){
            Log.e("ArmController", "Home Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.CarryToHome);
        }
        else if(gamePadState.y){
            Log.e("ArmController", "Carry Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.HomeToCarry);
        }
        else{
            //Log.i("ArmController", "Nothing detected in attack mode");
        }
    }

    private void checkArmLimits(Sensors sensors){
        if (sensors.baseLimit) {
            Log.e("ArmController: checkArmLimits", "Base Button Pressed!");
            godrick.arm.baseJoint.setTargetAngle(godrick.arm.baseJoint.getAngleDeg() + 1.0);
        }

        if (sensors.lowerLimitA) {
            Log.e("ArmController: checkArmLimits", "LowerA Button Pressed!");
            godrick.arm.elbowJoint.setTargetAngle(godrick.arm.elbowJoint.getAngleDeg() + 1.0);
        }

        else if (sensors.lowerLimitB) {
            Log.e("ArmController: checkArmLimits", "LowerB Button Pressed!");
            godrick.arm.elbowJoint.setTargetAngle(godrick.arm.elbowJoint.getAngleDeg() - 1.0);
        }
    }

//    private void findAngles(double x, double y) {
//        double distance = Math.sqrt(x*x+y*y);
//        double length1 = 0; // base.getPositionDistance(UnitOfDistance.CM);// TODO:Fix this ARM JOINT THING
//        double length2 = 0; //lower.getPositionDistance(UnitOfDistance.CM);// TODO:Fix this ARM JOINT THING
//        double extraAngle = UtilityKit.atan(y/x, UnitOfAngle.DEGREES);
//
//        double baseAngle = 90 - extraAngle - UtilityKit.acos((length1*length1+distance*distance-length2*length2)/(2*length1*distance), UnitOfAngle.DEGREES);
//        double lowerAngle = 180 - UtilityKit.acos((length1*length1+length2*length2-distance*distance)/(2*length1*length2), UnitOfAngle.DEGREES);
//
//        baseTicks = UtilityKit.armDegreesToTicks(baseAngle);
//        lowerTicks = UtilityKit.armDegreesToTicks(lowerAngle);
//    }

//    private Vector2D getTargetPosition(Sensors sensors) {
//        // TODO:Fix this ARM JOINT THING
//        double x = 0; // sensors.baseJointA.getPositionDistance(UnitOfDistance.CM)*UtilityKit.sin(UtilityKit.armTicksToDegrees(baseTicks), UnitOfAngle.DEGREES) + sensors.elbowJoint.getPositionDistance(UnitOfDistance.CM)*UtilityKit.sin(UtilityKit.armTicksToDegrees(lowerTicks), UnitOfAngle.DEGREES);
//        double y = 0; // sensors.baseJointA.getPositionDistance(UnitOfDistance.CM)*UtilityKit.cos(UtilityKit.armTicksToDegrees(baseTicks), UnitOfAngle.DEGREES) + sensors.elbowJoint.getPositionDistance(UnitOfDistance.CM)*UtilityKit.cos(UtilityKit.armTicksToDegrees(lowerTicks), UnitOfAngle.DEGREES);
//        return new Vector2D(x, y);
//    }
//
//    double startDistance;
//    Vector2D nextPosition;

}
