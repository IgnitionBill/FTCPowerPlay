package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name="MyAwesomeOpMode1", group = "Concept")
public class MyAwesomeOpMode extends LinearOpMode {
    private static final String VUFORIA_KEY =
            "ARxtv8D/////AAABmZ4ts8AsuUsfv570CA0FvZY2KnfvNO/V97gOg+9/vwscYSkdGKDVInMDgmTdCEI2e8l96txPXrBEK8uLhdOrPHdG4KePbI++PmIY30U7WO62l9+6kVQiW1Dqkc/ddlh9X4RkOGiadErsSDHFuE8sea4IieU+42L9BnIWJIvm9FeoMIpakxvy/e3TnHks4ZbKVkdImRnScYAX3X34Z3FknB6K6LfXwpk2MdDGQuFrZh/2M7u84uzDfSXt+Ltpv+VGO1+yxWu/+6rpzSp/sE3OrIF48kmwwRLCh8ixInK4S0R4f1vAFpgN2MI+h20H50j/Zp2c2ppY52Dzpq2Mk+f9JiWo90003D9syuWyCsxKzPUV";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        telemetry.addData("Status", "Webcam ready");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}