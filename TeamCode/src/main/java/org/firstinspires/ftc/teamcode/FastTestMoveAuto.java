package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;

@Autonomous(name = "FastForwardTest", group = "Auto")
public class FastTestMoveAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // Initialize
        DcMotorEx frontLeftDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx backLeftDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("backLeft");
        DcMotorEx frontRightDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("frontRight");
        DcMotorEx backRightDriveMotor = (DcMotorEx) this.hardwareMap.dcMotor.get("backRight");

        // Correct motor directions
        frontLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set drivetrain motors to brake on zero power for better stopping control
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoders to zero
        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Waiting for Play", "Wait for Referees and then Press Play");
        telemetry.update();
        waitForStart();

        // Run autonomous
        double startTime = this.time;

        double i = 0; // run number
        int previousTarget = 0; // backwards position
        int forwardDistance = UtilityKit.driveDistanceToTicks(90, UnitOfDistance.CM); // forwards position
        int currentTarget = forwardDistance; // current target position
        int currentPosition = 0; // as the name states
        boolean arrival = false; // arrival status

        frontLeftDriveMotor.setTargetPositionTolerance(10);
        frontRightDriveMotor.setTargetPositionTolerance(10);
        backRightDriveMotor.setTargetPositionTolerance(10);
        backLeftDriveMotor.setTargetPositionTolerance(10);

        while (opModeIsActive()) {
            if (i < 15) { // move robot five times
                arrival = false;

                // Move motors
                frontLeftDriveMotor.setTargetPosition(currentTarget);
                frontRightDriveMotor.setTargetPosition(currentTarget);
                backRightDriveMotor.setTargetPosition(currentTarget);
                backLeftDriveMotor.setTargetPosition(currentTarget);

                frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftDriveMotor.setPower(.5);
                backLeftDriveMotor.setPower(.5);
                frontRightDriveMotor.setPower(.5);
                backRightDriveMotor.setPower(.5);

                while (!arrival) { // update telemetry until we reach the destination
                    if (!frontLeftDriveMotor.isBusy() && !frontRightDriveMotor.isBusy() && !backRightDriveMotor.isBusy() && !backLeftDriveMotor.isBusy()) {
                        arrival = true;
                    }
                    currentPosition = (frontLeftDriveMotor.getCurrentPosition() + frontRightDriveMotor.getCurrentPosition() + backRightDriveMotor.getCurrentPosition() + backLeftDriveMotor.getCurrentPosition()) / 4;
                    telemetry.addData("Run number: ", i);
                    telemetry.addData("Run target: ", currentTarget);
                    telemetry.addData("Tick: ", currentPosition);
                    telemetry.addData("Arrival: ", arrival);
                    telemetry.update();
                }
                i++;
                // check if we should go forwards or backwards
                if ((int) (i / 2) == i / 2) {
                    currentTarget = forwardDistance;
                } else {
                    currentTarget = previousTarget;
                }
            }
        }
    }
}
