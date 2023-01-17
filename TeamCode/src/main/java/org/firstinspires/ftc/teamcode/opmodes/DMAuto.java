package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;

/**
 * AUTONOMOUS TO DO:
 * GOALS:
 * 1) Make a mode that just parks
 *      navigates to the parking spot
 * 2) Make a mode that Delivers the preloaded shipping element:
 *      unfolds the arm,
 *      drives forward,
 *      rotates to scan with range finder to detect the shipping element bar code,
 *      navigates to the shipping hub without colliding,
 *      moves the arm to the detected level,
 *      and deposits the block.
 *
 * STEPS:
 * 1) Develop basic navigation using the sensors:
 *      9-DOF
 *      Motor encoders
 *      Range finders
 *      Bumpers
 *    To maintain a sense of position and orientation,
 *    to avoid collisions with walls, game elements, other robots,
 *    to reduce impact by detecting unexpected touches on the bumpers,
 *    to move safely from the currently believed location to a specific destination
 *      a) Read each sensor and examine its telemetry to get a feel for the data provided
 *      b) Make a plan for how the sensors will be used
 *      c) Divide code development into small steps that can each be tested
 *      d) Build each part and test it
 *
 *  2) Develop parking mode using the basic navigation ability
 *  3) Develop duck spinning mode using navigation ability and sensors to detect the spinner
 *  4) Develop autonomous arm control (can also be used for tele-op)
 *  5) Develop a mode to deliver a preloaded block
 *  6) Put the modes together with variations for different strategies and play positions
 *      a) Consider starting from red (left or right) or blue (left or right)
 *      b) Consider backup modes for hardware failures during game day
 *      c) Any other options that might be useful for team strategy
 */
@Autonomous(name = "ForwardTest", group = "Auto")
public class DMAuto extends LinearOpMode {
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

        int forwardDistance = UtilityKit.driveDistanceToTicks(30, UnitOfDistance.CM);

        // Move forward as a test
        frontLeftDriveMotor.setTargetPosition(forwardDistance);
        frontRightDriveMotor.setTargetPosition(forwardDistance);
        backRightDriveMotor.setTargetPosition(forwardDistance);
        backLeftDriveMotor.setTargetPosition(forwardDistance);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDriveMotor.setPower(0.5);
        backLeftDriveMotor.setPower(0.5);
        frontRightDriveMotor.setPower(0.5);
        backRightDriveMotor.setPower(0.5);

        frontLeftDriveMotor.setTargetPositionTolerance(10);
        frontRightDriveMotor.setTargetPositionTolerance(10);
        backRightDriveMotor.setTargetPositionTolerance(10);
        backLeftDriveMotor.setTargetPositionTolerance(10);

        while (opModeIsActive() && (frontLeftDriveMotor.isBusy() || frontRightDriveMotor.isBusy())) {
            telemetry.addData("pos fl: ", frontLeftDriveMotor.getCurrentPosition());
            telemetry.addData("pos fr: ", frontRightDriveMotor.getCurrentPosition());
            telemetry.addData("pos br: ", backRightDriveMotor.getCurrentPosition());
            telemetry.addData("pos bl:", backLeftDriveMotor.getCurrentPosition());
            telemetry.update();
        }

        while(this.time - startTime < 1) {
            frontLeftDriveMotor.setPower(0.4);
            backLeftDriveMotor.setPower(0.4);
            frontRightDriveMotor.setPower(0.4);
            backRightDriveMotor.setPower(0.4);
        }
        frontLeftDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
    }
}
