package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.Godrick;

@TeleOp(name="GodrickTheOpMode", group = "FullOpMode")
public class GodrickTheOpMode extends LinearOpMode {

    // Create general variables
    private ElapsedTime runtime = new ElapsedTime();
    private Godrick godrick = new Godrick();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");

        telemetry.update();

        godrick.initialize(hardwareMap, gamepad1, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Running");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            godrick.update();
        }
    }
}
