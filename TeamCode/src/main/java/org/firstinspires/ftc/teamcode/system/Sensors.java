package org.firstinspires.ftc.teamcode.system;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.camera.CameraWrapper;
import org.firstinspires.ftc.teamcode.arm.ArmJoint;
import org.firstinspires.ftc.teamcode.arm.ArmReference;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.Vector2D;
import org.firstinspires.ftc.teamcode.util.Vector3D;

// Values recorded here are directly observed from sensors
public class Sensors {
    Telemetry telemetry;

    private boolean initialized = false;

    public Vector3D orientation = new Vector3D(0, 0, 0);
    public Vector2D position2D = new Vector2D();

    public Vector2D grabberPosition = new Vector2D();
    public Vector2D oldGrabberPosition = new Vector2D();
    public Vector2D deltaGrabberPosition = new Vector2D();

    // 9-DOF:
    // Magnetometer (indicates magnetic north, can be useful to correct gyroscope, but is subject to false magnetic fields)
    // Accelerometer (indicates robot acceleration)
    // Gyroscope (indicates robot orientation but may drift)
    private BNO055IMU gyro;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private Orientation angles;
    private Acceleration acceleration;
    private AngularVelocity angularVelocity;
    private Position position;

    // Time (time passed since the last data snapshot is useful for physics calculations)
    private ElapsedTime runtime = new ElapsedTime();

    // Touch Sensors
    private TouchSensor touchLowerA;
    private TouchSensor touchLowerB;
    private TouchSensor touchBase;
    private TouchSensor touchConeA;
    private TouchSensor touchConeB;
    private TouchSensor touchPole;

    public boolean lowerA;
    public boolean lowerB;
    public boolean base;
    public boolean coneA;
    public boolean coneB;
    public boolean pole;

    // Create delta position of drivetrain dc motors
    public int deltaFrontLeftPosition;
    public int deltaFrontRightPosition;
    public int deltaBackRightPosition;
    public int deltaBackLeftPosition;

    // Create old positions of drivetrain dc motors
    public int oldFrontLeftPosition;
    public int oldFrontRightPosition;
    public int oldBackRightPosition;
    public int oldBackLeftPosition;

    // Create current positions of drivetrain dc motors
    public int frontLeftPosition;
    public int frontRightPosition;
    public int backRightPosition;
    public int backLeftPosition;

    public ArmJoint turnData;
    public ArmJoint baseData;
    public ArmJoint baseDataB;
    public ArmJoint lowerData;

    public ArmPose currentPose;

    private double time = 0;
    private double lastTime = 0;
    private double dt = 0; // the time interval since the last update function call

    public double getDt(){
        return dt;
    }
    public double getTime() { return time; }

    public CameraWrapper cameraWrapper;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;
        cameraWrapper = new CameraWrapper(hardwareMap.appContext);
        // TODO: Coach suggests these data elements are a better fit in the ArmController class
        // TODO: Coach suggests not to use the nautical convention here and to use positive and negative values for min and max angles, and call them min and max, min < max is obvious
        //TODO: Set angleOffset to the default angle the joints start at
        //TODO: Set proper x/y coordinates
        turnData = new ArmJoint("TurntableJoint", UnitOfAngle.DEGREES, 180, ArmReference.PORT, 180, ArmReference.STARBOARD, 20, 15, UnitOfDistance.CM);
        baseData = new ArmJoint("BaseJointA", UnitOfAngle.DEGREES, 90, ArmReference.BOW, 80, ArmReference.STERN, 2.4, 28.8, UnitOfDistance.CM);
        baseDataB = new ArmJoint("BaseJointB", UnitOfAngle.DEGREES, 90, ArmReference.BOW, 80, ArmReference.STERN, 2.4, 28.8, UnitOfDistance.CM);
        lowerData = new ArmJoint("LowerJoint", UnitOfAngle.DEGREES, 170, ArmReference.BOW, 0, ArmReference.STERN, -2.4, 28.8, UnitOfDistance.CM);

        currentPose = new ArmPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        try {
            //TODO: Determine whether IMU is required
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            gyro.initialize(parameters);

            touchLowerA = hardwareMap.get(TouchSensor.class, "lowerTouchA");
            touchLowerB = hardwareMap.get(TouchSensor.class, "lowerTouchB");
            touchBase = hardwareMap.get(TouchSensor.class, "baseLimit");
            touchConeA = hardwareMap.get(TouchSensor.class, "coneTouchA");
            touchConeB = hardwareMap.get(TouchSensor.class, "coneTouchB");
            touchPole = hardwareMap.get(TouchSensor.class, "poleTouch");

            runtime.reset();

            initialized = true;
        }
        catch (Exception e){
            telemetry.addData("Sensor Init Failed! ", e.toString());
            telemetry.update();
        }
    }

    public void update(Actuators actuators, boolean verbose) {
        try {
            time = runtime.seconds();
            dt = time - lastTime;
            lastTime = time;

            lowerData.updateAngles(actuators.lowerSegment.getCurrentPosition());
            baseData.updateAngles(actuators.baseSegment.getCurrentPosition());
            baseDataB.updateAngles(actuators.baseSegment2.getCurrentPosition());
            turnData.updateAngles(actuators.turnTable.getCurrentPosition());

            lowerData.updateSpeed(actuators.lowerSegment.getVelocity());
            baseData.updateSpeed(actuators.baseSegment.getVelocity());
            baseDataB.updateSpeed(actuators.baseSegment2.getVelocity());
            turnData.updateSpeed(actuators.turnTable.getVelocity());

            currentPose.setThetas(
                    turnData.getCurrentAngle(UnitOfAngle.DEGREES),
                    baseData.getCurrentAngle(UnitOfAngle.DEGREES),
                    lowerData.getCurrentAngle(UnitOfAngle.DEGREES),
                    0,
                    0,
                    0,
                    0
            );

            currentPose.setOmegas(
                    turnData.getCurrentSpeed(UnitOfAngle.DEGREES),
                    baseData.getCurrentSpeed(UnitOfAngle.DEGREES),
                    lowerData.getCurrentSpeed(UnitOfAngle.DEGREES),
                    0,
                    0,
                    0,
                    0
            );
            // Set old positions for drivetrain dc motors
            oldFrontLeftPosition = frontLeftPosition;
            oldFrontRightPosition = frontRightPosition;
            oldBackRightPosition = backRightPosition;
            oldBackLeftPosition = backLeftPosition;

            // Set current positions of drivetrain dc motors
            frontLeftPosition = actuators.getFrontLeftPosition();
            frontRightPosition = actuators.getFrontRightPosition();
            backRightPosition = actuators.getBackRightPosition();
            backLeftPosition = actuators.getBackLeftPosition();

            // Set delta positions of drivetrain dc motors
            deltaFrontLeftPosition = frontLeftPosition - oldFrontLeftPosition;
            deltaFrontRightPosition = frontRightPosition - oldFrontRightPosition;
            deltaBackRightPosition = backRightPosition - oldBackRightPosition;
            deltaBackLeftPosition = backLeftPosition - oldBackLeftPosition;

            angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = formatAngle(angles.angleUnit, angles.firstAngle);
            double roll  = formatAngle(angles.angleUnit, angles.secondAngle);
            double pitch = formatAngle(angles.angleUnit, angles.thirdAngle);
            orientation.set(pitch, roll, heading);

            acceleration = gyro.getOverallAcceleration();
            angularVelocity = gyro.getAngularVelocity();
            position = gyro.getPosition();//.toUnit(DistanceUnit.INCH);
            position2D.set(position.x, position.y);

            lowerA = touchLowerA.isPressed();
            lowerB = touchLowerB.isPressed();
            base = touchBase.isPressed();
            coneA = touchConeA.isPressed();
            coneB = touchConeB.isPressed();
            pole = touchPole.isPressed();

            oldGrabberPosition = grabberPosition;
            double grabberX = baseData.getX(UnitOfDistance.CM)+lowerData.getX(UnitOfDistance.CM);
            double grabberY = baseData.getY(UnitOfDistance.CM)+lowerData.getY(UnitOfDistance.CM);
            grabberPosition.set(grabberX, grabberY);
            deltaGrabberPosition.set(grabberPosition.getX()-oldGrabberPosition.getX(), grabberPosition.getY()-oldGrabberPosition.getY());

            if(verbose) {
                telemetry.addData("Position: ", position.toString());
                telemetry.addData("Orientation: ", angles.toString());
                telemetry.addData("Turn: ", actuators.turnTable.getCurrentPosition());
                telemetry.addData("Base: ", actuators.baseSegment.getCurrentPosition());
                telemetry.addData("BaseB: ", actuators.baseSegment2.getCurrentPosition());
                telemetry.addData("Lower: ", actuators.lowerSegment.getCurrentPosition());

                // buttons
                telemetry.addData("lower limit A: ", lowerA);
                telemetry.addData("lower Limit B: ", lowerB);
                telemetry.addData("pole touch: ", pole); // no, no
                telemetry.addData("base limit: ", base);
                telemetry.addData("cone A: ", coneA); // no, light
                telemetry.addData("cone B: ", coneB);
            }
        }

        catch (Exception e) {
            telemetry.addData("Sensor failure ", e.toString());
        }
    }

    Double formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    Double formatDegrees(double degrees){
        return AngleUnit.DEGREES.normalize(degrees);
    }
}
