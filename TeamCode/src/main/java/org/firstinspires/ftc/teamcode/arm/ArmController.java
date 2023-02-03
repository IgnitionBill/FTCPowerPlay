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
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;

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
    boolean turnTableHome = false;

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
        turnTableHome = false;

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
        else {
            if (!godrick.gamePadState.altMode) {
                armControlMode = ArmControlMode.ATTACK;
            }
            if (godrick.gamePadState.altMode) {
                armControlMode = ArmControlMode.MANUAL;
            }
        }
        if (armControlMode == ArmControlMode.ATTACK) {
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

        CylindricalVector3D currentPosition;

        if(gamePadState.dPadUp){
            // move the arm gripper forward via inverse kinematics
            // x += diff;
//            if (currentPosition.rho < godrick.arm.maxArmHeight-5) {
//                currentPosition.rho++;
//            }
        }
        if(gamePadState.dPadDown){
            // move the arm gripper backward via inverse kinematics
            // x -= diff;
//            if (currentPosition.rho > 10) {
//                currentPosition.rho--;
//            }
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

//        if (Math.sqrt(currentPosition.rho) < godrick.arm.maxArmHeight) {
//
//        }
    }

    // TODO: add autohome for turntable
    private void autoHome(Sensors sensors, Actuators actuators, boolean verbose) {
        Log.i("ArmController", "Running autoHome");

        // after both buttons have been pressed, autohome complete, next mode
        if (baseHome && lowerHome && turnTableHome) {
            armControlMode = ArmControlMode.ATTACK;
            Log.e("ArmController", "autoHome complete");
        }
        // until both buttons have been pressed,
        else { // slowly move each arm toward the button
            if (!baseHome) {
                godrick.arm.baseJoint.incrementTargetAngle(-2.0);
                //godrick.arm.baseJoint.setTargetAngle(godrick.arm.baseJoint.getAngleDeg() - 1.0);
            }
            if(!lowerHome) {
                godrick.arm.elbowJoint.incrementTargetAngle(2.0);
                //godrick.arm.elbowJoint.setTargetAngle(godrick.arm.elbowJoint.getAngleDeg() + 1.0);
            }
            if(!turnTableHome){
                godrick.arm.turntable.incrementTargetAngle(2.0);
            }
        }

        if (verbose) {
            telemetry.addData("ArmControlMode ", armControlMode.toString());
        }
    }

    private void godrickDemo(){
        motionSequenceDirector.update();
    }

    // when the B button is pressed, it looks for a pole and places it
    private void godrickAttackMode(GamePadState gamePadState, Sensors sensors){
        // update the motion director
        motionSequenceDirector.update();

        if(gamePadState.a){
            Log.e("ArmController", "Grab Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.GrabCone);
        }
        else if(gamePadState.b){
            Log.e("ArmController", "Place Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.PlaceCone);
        }
        else if(gamePadState.x){
            Log.e("ArmController", "ConeView Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.ToConeView);
        }
        else if(gamePadState.y){
            Log.e("ArmController", "Carry Pressed.");
            motionSequenceDirector.requestNewSequence(MotionSequenceName.ToCarry);
        }
        else{
            //Log.i("ArmController", "Nothing detected in attack mode");
        }
    }

    private void checkArmLimits(Sensors sensors){
        if (sensors.baseLimit) {
            Log.e("ArmController: checkArmLimits", "Base Button Pressed!");
            baseHome = true;
            actuators.baseSegment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            actuators.baseSegment2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            godrick.arm.baseJoint.setTargetToHome();
            godrick.arm.baseJoint.incrementTargetAngle(8.0);
        }

        if (sensors.lowerLimitA) {
            Log.e("ArmController: checkArmLimits", "LowerA Button Pressed!");
            lowerHome = true;
            actuators.lowerSegment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            godrick.arm.elbowJoint.setTargetToMin();
            godrick.arm.elbowJoint.incrementTargetAngle(8.0);
        }

        else if (sensors.lowerLimitB) {
            Log.e("ArmController: checkArmLimits", "LowerB Button Pressed!");
            lowerHome = true;
            actuators.lowerSegment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            godrick.arm.elbowJoint.setTargetToHome();
            godrick.arm.elbowJoint.incrementTargetAngle(-8.0);
        }

        // todo: only stop to reset when the position is off by a lot
        if(!sensors.tableLimit){
            if(armControlMode == ArmControlMode.AUTO_HOME) {
                //Log.e("ArmController: checkArmLimits", "Turn Table Limit Reached");
                turnTableHome = true;
                actuators.turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                godrick.arm.turntable.setTargetToHome();
                godrick.arm.turntable.incrementTargetAngle(0);
            }
        }
    }
}
