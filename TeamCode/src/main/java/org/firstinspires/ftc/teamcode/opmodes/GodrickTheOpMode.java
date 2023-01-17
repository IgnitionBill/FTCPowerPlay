package org.firstinspires.ftc.teamcode.opmodes;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CameraWrapper;
import org.firstinspires.ftc.teamcode.system.Actuators;
import org.firstinspires.ftc.teamcode.system.GamePadState;
import org.firstinspires.ftc.teamcode.drivetrain.MotorController;
import org.firstinspires.ftc.teamcode.system.SafetyMonitor;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.arm.DefinedArmPositions;
import org.firstinspires.ftc.teamcode.arm.DefinedMotionSequences;

@TeleOp(name="GodrickTheOpMode", group = "FullOpMode")

public class GodrickTheOpMode extends LinearOpMode {

    //TODO: Cleanup

    // Create general variables
    private ElapsedTime runtime = new ElapsedTime();

    // Create references to sensor classes
    private GamePadState gamePadState = new GamePadState();
    private Actuators actuators = new Actuators();
    private Sensors sensors = new Sensors();
    private SafetyMonitor safetyMonitor = new SafetyMonitor();
    private ArmController armController = new ArmController();
    private DefinedArmPositions definedArmPositions = new DefinedArmPositions();
    private DefinedMotionSequences definedMotionSequences = new DefinedMotionSequences();

    // Create references to control classes
    private MotorController motorController = new MotorController();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //sensoryState.initialize(hardwareMap, telemetry);
        actuators.initializeGodrick(hardwareMap, telemetry);
        sensors.initialize(hardwareMap, telemetry);
        motorController.initialize(telemetry);
        armController.initialize(sensors, telemetry);
        definedMotionSequences.init(definedArmPositions);
        gamePadState.initialize(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Running");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Alt mode:", gamePadState.altMode);
            // store the latest gamepad state
            gamePadState.update(gamepad1, false);
            // update the sensors with data from the actuators
            sensors.update(actuators, true);

            // update the motor controller state, to make the motors move
            motorController.simpleMechanumUpdate(gamePadState, sensors, false);

            // Calculate the next move for the servo motors
            motorController.servoUpdate(gamePadState);

            //Calculate the next move for the DC motors
            armController.updateArm(gamePadState, actuators, sensors, true);

            actuators.updateDrivetrainMotors(motorController);
            actuators.updateServos(motorController);
            actuators.updateArm(armController, sensors);

            // display all telemetry updates to the controller, use verbose=true to see reports in telemetry
            telemetry.update();
        }
    }
}
