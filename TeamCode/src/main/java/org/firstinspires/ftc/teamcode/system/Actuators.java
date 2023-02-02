package org.firstinspires.ftc.teamcode.system;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.MechanumController;
import org.firstinspires.ftc.teamcode.util.UtilityKit;

public class Actuators {
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // Declare drivetrain dc motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx backLeft;

    // Declare joint controllers for arm
    public DcMotorEx turnTable;
    public DcMotorEx baseSegment;
    public DcMotorEx baseSegment2;
    public DcMotorEx lowerSegment;

    // Declare servo control for arm
    public Servo grabberServo; // th6
    public Servo rollServo; // th3
    public Servo yawServo; // th4
    public Servo pitchServo; // th5

    Godrick godrick;

    // Initialize GodrickBot
    public void initializeGodrick(Godrick godrick) {
        this.godrick = godrick;
        telemetry = godrick.telemetry;
        hardwareMap = godrick.hardwareMap;

        try {
            // Initialize drivetrain dc motors
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e) {
            Log.e("Actuators", "Drivetrain dc motors failed to initialize");
            Log.e("Actuators", e.toString());
        }

        try {
            // Initialize arm dc motors
            turnTable = hardwareMap.get(DcMotorEx.class, "turnTable");
            baseSegment = hardwareMap.get(DcMotorEx.class, "baseSegment");
            baseSegment2 = hardwareMap.get(DcMotorEx.class, "baseSegment2");
            lowerSegment = hardwareMap.get(DcMotorEx.class, "lowerSegment");
            lowerSegment.setDirection(DcMotorSimple.Direction.REVERSE);

            turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseSegment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseSegment2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lowerSegment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set the target position of the arm to the target position TODO: COMMENTED THIS BECAUSE WE DON'T WANT TO MOVE YET
//            turnTable.setTargetPosition(UtilityKit.armDegreesToTicks(godrick.arm.HOME_TH0));
//            baseSegment.setTargetPosition(UtilityKit.armDegreesToTicks(godrick.arm.MIN_TH1));
//            baseSegment2.setTargetPosition(UtilityKit.armDegreesToTicks(godrick.arm.MIN_TH1));
//            lowerSegment.setTargetPosition(UtilityKit.armDegreesToTicks(godrick.arm.MAX_TH2));
//
//            turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            baseSegment.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            baseSegment2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lowerSegment.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turnTable.setTargetPositionTolerance(2);
            baseSegment.setTargetPositionTolerance(2);
            baseSegment2.setTargetPositionTolerance(2);
            lowerSegment.setTargetPositionTolerance(2);
        }
        catch (Exception e) {
            Log.e("Actuators: initialize", "Arm dc motors failed to initialize");
            Log.e("Actuators: initialize", e.toString());
        }

        try {
            // Initialize arm servo motors
            grabberServo = hardwareMap.get(Servo.class, "finger");
            rollServo = hardwareMap.get(Servo.class, "roll");
            yawServo = hardwareMap.get(Servo.class, "yaw");
            pitchServo = hardwareMap.get(Servo.class, "pitch");
        }
        catch (Exception e) {
            System.out.println("Servos failed to initialize");
            Log.e("Actuators", "Servo motors failed to initialize");
            Log.e("Actuators", e.toString());
        }
    }

    // get methods for drivetrain dc motors
    public int getFrontLeftPosition() {return frontLeft.getCurrentPosition();}
    public int getFrontRightPosition() {return frontRight.getCurrentPosition();}
    public int getBackRightPosition() {return backRight.getCurrentPosition();}
    public int getBackLeftPosition() {return backLeft.getCurrentPosition();}

    public void updateDrivetrainMotors(MechanumController mechanumController) {
        // set target positions for drivetrain dc motors
        frontLeft.setTargetPosition(mechanumController.frontLeftTicks);
        frontRight.setTargetPosition(mechanumController.frontRightTicks);
        backRight.setTargetPosition(mechanumController.backRightTicks);
        backLeft.setTargetPosition(mechanumController.backLeftTicks);

        // set power for drivetrain dc motors
        frontLeft.setPower(mechanumController.frontLeft);
        frontRight.setPower(mechanumController.frontRight);
        backRight.setPower(mechanumController.backRight);
        backLeft.setPower(mechanumController.backLeft);

        // set run mode for drivetrain dc motors
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // this is for running to a target velocity
    }

    public void updateArm() {
        StringBuilder sb = new StringBuilder();
        StringBuilder sb2 = new StringBuilder();

        turnTable.setTargetPosition(godrick.arm.turntable.getTargetTicks());
        baseSegment.setTargetPosition(godrick.arm.baseJoint.getTargetTicks());
        baseSegment2.setTargetPosition(godrick.arm.baseJoint.getTargetTicks());
        lowerSegment.setTargetPosition(godrick.arm.elbowJoint.getTargetTicks());

        turnTable.setPower(1.0);
        baseSegment.setPower(1.0);
        baseSegment2.setPower(1.0);
        lowerSegment.setPower(1.0);

        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseSegment.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseSegment2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowerSegment.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sb2.append("deg: ");
        sb2.append(" th0 " + godrick.arm.turntable.toString());
        sb2.append(" th1 " + godrick.arm.baseJoint.toString());
        sb2.append(" th2 " + godrick.arm.elbowJoint.toString());
        Log.i("Actuators: updateArm", sb2.toString());

        sb.append("ticks: ");
        sb.append(" th0 current: " + turnTable.getCurrentPosition());
        sb.append(" target: " + turnTable.getTargetPosition() + " ");
        sb.append(" th1a current: " + baseSegment.getCurrentPosition());
        sb.append(" target: " + baseSegment.getTargetPosition() + " ");
        sb.append(" th1b current: " + baseSegment2.getCurrentPosition());
        sb.append(" target: " + baseSegment2.getTargetPosition() + " ");
        sb.append(" th2 current: " + lowerSegment.getCurrentPosition());
        sb.append(" target: " + lowerSegment.getTargetPosition() + " ");
        Log.i("Actuators: updateArm", sb.toString());

        // GoBilda 2000-0025-0002 300 degree max rotation
        rollServo.setPosition((1/150.0) * godrick.arm.grabberRoll+.5);
        pitchServo.setPosition((1/150.0) * godrick.arm.grabberPitch+.5);
        yawServo.setPosition((1/150.0)* godrick.arm.grabberYaw+.5);
        grabberServo.setPosition((1/150.0) * godrick.arm.grabberGrip+.5);

        //Log.e("Actuators: updateArm", sb.toString());

        // TODO: what do we set the joint velocity to and why?
//        turnTable.setVelocity(UtilityKit.ticksPerDegreeAtJoint*30); // why do we set the velocity?
//        baseSegment.setVelocity(UtilityKit.ticksPerDegreeAtJoint*30);
//        baseSegment2.setVelocity(UtilityKit.ticksPerDegreeAtJoint*30);
//        lowerSegment.setVelocity(UtilityKit.ticksPerDegreeAtJoint*30);

    }

//    public void updateServos(MotorController motorController) {
//        if (motorController.grabberRelease) {
//            grabberServo.setPosition(1);
//        }
//        else {
//            grabberServo.setPosition(-1);
//        }
//    }
}
