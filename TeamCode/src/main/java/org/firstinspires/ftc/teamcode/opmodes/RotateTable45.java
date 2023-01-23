package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.UtilityKit;

@Autonomous(name = "RotateTableCounter45", group = "Auto")
public class RotateTable45 extends LinearOpMode {
    // Create general variables
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        DcMotorEx turnTable = (DcMotorEx) this.hardwareMap.dcMotor.get("turnTable");

        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turnTable.setPower(.5);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Waiting for Play", "Wait for Referees and then Press Play");
        telemetry.update();
        waitForStart();

        runtime.reset();

        turnTable.setTargetPosition(UtilityKit.armDegreesToTicks(45));
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            telemetry.addData("Turning! ", true);
            telemetry.update();
        }
    }
}
