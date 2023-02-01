package org.firstinspires.ftc.teamcode.system;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.MechanumController;
import org.firstinspires.ftc.teamcode.drivetrain.MotorController;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.arm.JointController;
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
    public JointController turnTable;
    public JointController baseSegment;
    public JointController baseSegment2;
    public JointController lowerSegment;

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
            turnTable = new JointController(hardwareMap, "turnTable");
            lowerSegment = new JointController(hardwareMap, "lowerSegment");
            lowerSegment.reverse();
            baseSegment = new JointController(hardwareMap, "baseSegment");
            baseSegment2 = new JointController(hardwareMap, "baseSegment2");

            turnTable.setPower(1.0);
            baseSegment.setPower(1.0);
            baseSegment2.setPower(1.0);
            lowerSegment.setPower(1.0);
        }
        catch (Exception e) {
            System.out.println("Arm dc motors failed to initialize");
            Log.e("Actuators", "Arm dc motors failed to initialize");
            Log.e("Actuators", e.toString());
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
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateArm(ArmController armController, Sensors sensors) {
        StringBuilder sb = new StringBuilder();

        turnTable.setTarget(godrick.arm.turntable.getTargetTicks());
        baseSegment.setTarget(godrick.arm.baseJointA.getTargetTicks());
        baseSegment2.setTarget(godrick.arm.baseJointB.getTargetTicks());
        lowerSegment.setTarget(godrick.arm.elbowJoint.getTargetTicks());
        sb.append(turnTable.deviceName + " current: ");
        sb.append(turnTable.getCurrentPosition());

        // GoBilda 2000-0025-0002 300 degree max rotation
        rollServo.setPosition((1/150.0) * godrick.arm.grabberRoll+.5);
        pitchServo.setPosition((1/150.0) * godrick.arm.grabberPitch+.5);
        yawServo.setPosition((1/150.0)* godrick.arm.grabberYaw+.5);
        grabberServo.setPosition((1/150.0) * godrick.arm.grabberGrip+.5);

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
