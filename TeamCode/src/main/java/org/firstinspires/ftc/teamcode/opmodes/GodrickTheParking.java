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
import org.firstinspires.ftc.teamcode.system.Sensors;
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

    private static final String TFOD_MODEL_ASSET = "GOOSE.tflite";

    private static final String[] LABELS = {
            "DuckOne",
            "DuckTwo",
            "DuckThree"
    };

    private static final String VUFORIA_KEY =
            "ARxtv8D/////AAABmZ4ts8AsuUsfv570CA0FvZY2KnfvNO/V97gOg+9/vwscYSkdGKDVInMDgmTdCEI2e8l96txPXrBEK8uLhdOrPHdG4KePbI++PmIY30U7WO62l9+6kVQiW1Dqkc/ddlh9X4RkOGiadErsSDHFuE8sea4IieU+42L9BnIWJIvm9FeoMIpakxvy/e3TnHks4ZbKVkdImRnScYAX3X34Z3FknB6K6LfXwpk2MdDGQuFrZh/2M7u84uzDfSXt+Ltpv+VGO1+yxWu/+6rpzSp/sE3OrIF48kmwwRLCh8ixInK4S0R4f1vAFpgN2MI+h20H50j/Zp2c2ppY52Dzpq2Mk+f9JiWo90003D9syuWyCsxKzPUV";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void runOpMode() throws InterruptedException {
        Sensors sensors = new Sensors();
        sensors.initialize(hardwareMap, telemetry);

        initVuforia();
       // initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        MechanumController mechanumController = new MechanumController();

        //DriveMove forward = mechanumController.moveInDirection(27.5, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "forward");
        //DriveMove secondForward = mechanumController.moveInDirection(12, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "secondForward");
        //DriveMove moveToCenter = mechanumController.moveInDirection(40, UnitOfDistance.IN, -90, UnitOfAngle.DEGREES, "moveToCenter");
        //DriveMove left = mechanumController.moveInDirection(16, UnitOfDistance.IN, 90, UnitOfAngle.DEGREES, "left");
        //DriveMove middle  = mechanumController.moveInDirection(40, UnitOfDistance.IN, 90, UnitOfAngle.DEGREES, "middle");
        //DriveMove right = mechanumController.moveInDirection(68, UnitOfDistance.IN, 90, UnitOfAngle.DEGREES, "right");

        DriveMove setupForward = mechanumController.moveInDirection(4, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "setupMove");
        DriveMove forward = mechanumController.moveInDirection(27.5-4, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "forward");
        DriveMove secondForward = mechanumController.moveInDirection(12, UnitOfDistance.IN, 0, UnitOfAngle.DEGREES, "secondForward");
        DriveMove left = mechanumController.moveInDirection(27, UnitOfDistance.IN, -90, UnitOfAngle.DEGREES, "left");
        DriveMove right = mechanumController.moveInDirection(27, UnitOfDistance.IN, 90, UnitOfAngle.DEGREES, "right");

        ArrayList<DriveMove> leftPark = new ArrayList<>();
        leftPark.add(forward);
        //leftPark.add(moveToCenter);
        leftPark.add(left);
        leftPark.add(secondForward);

        ArrayList<DriveMove> middlePark = new ArrayList<>();
        middlePark.add(forward);
        //middlePark.add(moveToCenter);
        //middlePark.add(middle);
        middlePark.add(secondForward);

        ArrayList<DriveMove> rightPark = new ArrayList<>();
        rightPark.add(forward);
        //rightPark.add(moveToCenter);
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
        double startTime = this.time;
        boolean searching = true;
        int previousCheck = 0;
        ArrayList<String> collectedLabels = new ArrayList<>();

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

        // Find parking then set sequence
        while (opModeIsActive() && searching) {
            telemetry.addData("Setup: ", " Searching for object");

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                        if (runtime.time(TimeUnit.SECONDS) > previousCheck) {
                            collectedLabels.add(recognition.getLabel());

                        }
                    }

                    telemetry.addData("Time: ", runtime.time(TimeUnit.SECONDS));
                    telemetry.addData("Most common label: ", mostCommonLabel(collectedLabels));
                    telemetry.update();
                }
            }

            // when enough time has passed exit loop
            if (runtime.time(TimeUnit.SECONDS) > 5) {
                searching = false;
            }
        }

        // find most common label
        String commonLabel = mostCommonLabel(collectedLabels);
        ArrayList<DriveMove> currentSequence;

        // set sequence to correspond to the label
        if (commonLabel == LABELS[0]) {
            currentSequence = leftPark;
        } else if (commonLabel == LABELS[1]) {
            currentSequence = middlePark;
        } else if (commonLabel == LABELS[2]) {
            currentSequence = rightPark;
        } else {
            currentSequence = middlePark;
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
        frontLeftDriveMotor.setTargetPositionTolerance(10);
        frontRightDriveMotor.setTargetPositionTolerance(10);
        backRightDriveMotor.setTargetPositionTolerance(10);
        backLeftDriveMotor.setTargetPositionTolerance(10);

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

    private String mostCommonLabel(ArrayList<String> list) {
        String mostCommon = "NO LABELS";
        int maxCount = 0;

        for (int i = 0; i < list.size(); i++) {
            int count = 0;

            for (int j = 0; j < list.size(); j++) {
                if (list.get(i) == list.get(j)) {
                    count++;
                }
            }

            if (count > maxCount) {
                maxCount = count;
                mostCommon = list.get(i);
            }
        }

        return mostCommon;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
