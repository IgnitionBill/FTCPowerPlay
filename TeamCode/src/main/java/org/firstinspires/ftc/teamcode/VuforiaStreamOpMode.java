package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "ARxtv8D/////AAABmZ4ts8AsuUsfv570CA0FvZY2KnfvNO/V97gOg+9/vwscYSkdGKDVInMDgmTdCEI2e8l96txPXrBEK8uLhdOrPHdG4KePbI++PmIY30U7WO62l9+6kVQiW1Dqkc/ddlh9X4RkOGiadErsSDHFuE8sea4IieU+42L9BnIWJIvm9FeoMIpakxvy/e3TnHks4ZbKVkdImRnScYAX3X34Z3FknB6K6LfXwpk2MdDGQuFrZh/2M7u84uzDfSXt+Ltpv+VGO1+yxWu/+6rpzSp/sE3OrIF48kmwwRLCh8ixInK4S0R4f1vAFpgN2MI+h20H50j/Zp2c2ppY52Dzpq2Mk+f9JiWo90003D9syuWyCsxKzPUV";;

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}