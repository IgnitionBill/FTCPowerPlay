package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drivetrain.DriveMove;
import org.firstinspires.ftc.teamcode.drivetrain.MechanumController;
import org.firstinspires.ftc.teamcode.system.EldenParkingSpotWebcam;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.ParkingEnum;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "GodrickTheParking", group = "Auto")
public class GodrickTheParking extends LinearOpMode {

    // Create general variables
    private ElapsedTime runtime = new ElapsedTime();
    EldenParkingSpotWebcam eldenParkingSpotWebcam;
    ParkingEnum target;
    public static final String webcam = "Webcam 1";

    public void runOpMode() throws InterruptedException {
        eldenParkingSpotWebcam = new EldenParkingSpotWebcam(telemetry, hardwareMap, runtime, webcam);

        MechanumController mechanumController = new MechanumController();

        DriveMove setupForward = mechanumController.moveInDirection(4, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "setupMove");
        DriveMove forward = mechanumController.moveInDirection(27.5-4, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "forward");
        DriveMove secondForward = mechanumController.moveInDirection(12, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "secondForward");
        DriveMove left = mechanumController.moveInDirection(27, UnitOfDistance.IN, -90, UnitOfAngle.DEGREES, "left");
        DriveMove right = mechanumController.moveInDirection(27, UnitOfDistance.IN, 90, UnitOfAngle.DEGREES, "right");

        ArrayList<DriveMove> leftPark = new ArrayList<>();
        leftPark.add(forward);
        leftPark.add(left);
        leftPark.add(secondForward);

        ArrayList<DriveMove> middlePark = new ArrayList<>();
        middlePark.add(forward);
        //middlePark.add(middle);
        middlePark.add(secondForward);

        ArrayList<DriveMove> rightPark = new ArrayList<>();
        rightPark.add(forward);
        rightPark.add(right);
        rightPark.add(secondForward);

        // Initialize
        DcMotorEx frontLeftDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx backLeftDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("backLeft");
        DcMotorEx frontRightDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("frontRight");
        DcMotorEx backRightDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("backRight");

        DcMotorEx turnTable = (DcMotorEx) this.hardwareMap.dcMotor.get("turnTable");

        // Correct motor directions
        frontLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set drivetrain motors to brake on zero power for better stopping control
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoders to zero
        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDriveMotor.setPower(.5);
        backLeftDriveMotor.setPower(.5);
        frontRightDriveMotor.setPower(.5);
        backRightDriveMotor.setPower(.5);

        turnTable.setPower(1);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Waiting for Play", "Wait for Referees and then Press Play");
        telemetry.update();
        waitForStart();

        runtime.reset();

        // Run autonomous
        turnTable.setTargetPosition(UtilityKit.armDegreesToTicks(-45));
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDriveMotor.setTargetPosition(setupForward.frontLeftTicks);
        frontRightDriveMotor.setTargetPosition(setupForward.frontRightTicks);
        backRightDriveMotor.setTargetPosition(setupForward.backRightTicks);
        backLeftDriveMotor.setTargetPosition(setupForward.backLeftTicks);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && turnTable.isBusy()) {
            telemetry.addData("Setup: ", " Turning turnTable");
        }

        double waitTime = runtime.seconds()+2.5;
        while (runtime.seconds() < waitTime) {
            target = eldenParkingSpotWebcam.getParkingSpot();
        }

        ArrayList<DriveMove> currentSequence = new ArrayList<>();
        switch (target) {
            case PARK1: currentSequence = leftPark; break;
            case PARK2: currentSequence = middlePark; break;
            case PARK3: currentSequence = rightPark; break;
        }

        // create sequence variables
        boolean arrival = false;
        int sequenceIndex = 0;
        String moveName = "Nameless";

        int frontLeftTicks = 0;
        int frontRightTicks = 0;
        int backRightTicks = 0;
        int backLeftTicks = 0;

        // set position tolerance
        frontLeftDriveMotor.setTargetPositionTolerance(5);
        frontRightDriveMotor.setTargetPositionTolerance(5);
        backRightDriveMotor.setTargetPositionTolerance(5);
        backLeftDriveMotor.setTargetPositionTolerance(5);

        // Run required drive sequence
        while (opModeIsActive()) {
            frontLeftTicks = frontLeftDriveMotor.getCurrentPosition();
            frontRightTicks = frontRightDriveMotor.getCurrentPosition();
            backRightTicks = backRightDriveMotor.getCurrentPosition();
            backLeftTicks = backLeftDriveMotor.getCurrentPosition();



            if (!frontLeftDriveMotor.isBusy()) {
                if (!frontRightDriveMotor.isBusy()) {
                    if (!backRightDriveMotor.isBusy()) {
                        if (!backLeftDriveMotor.isBusy()) {
                            if (sequenceIndex < currentSequence.size()) {
                                frontLeftDriveMotor.setTargetPosition(currentSequence.get(sequenceIndex).frontLeftTicks + frontLeftTicks);
                                frontRightDriveMotor.setTargetPosition(currentSequence.get(sequenceIndex).frontRightTicks + frontRightTicks);
                                backRightDriveMotor.setTargetPosition(currentSequence.get(sequenceIndex).backRightTicks + backRightTicks);
                                backLeftDriveMotor.setTargetPosition(currentSequence.get(sequenceIndex).backLeftTicks + backLeftTicks);

                                frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                moveName = currentSequence.get(sequenceIndex).moveName;

                                sequenceIndex++;
                            }
                            else {
                                arrival = true;
                            }
                        }
                    }
                }
            }

            telemetry.addData("frontLeft: ", frontLeftTicks);
            telemetry.addData("frontRight: ", frontRightTicks);
            telemetry.addData("backRight: ", backRightTicks);
            telemetry.addData("backLeft: ", backLeftTicks);
            telemetry.addData("sequenceIndex: ", sequenceIndex);
            telemetry.addData("sequence size: ", currentSequence.size());
            telemetry.addData("Move name:", moveName);
            telemetry.addData("Arrival: ", arrival);
            telemetry.update();
        }
    }
}
