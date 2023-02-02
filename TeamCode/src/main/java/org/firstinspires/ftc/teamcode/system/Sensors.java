package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import org.firstinspires.ftc.teamcode.CameraWrapper;
import org.firstinspires.ftc.teamcode.util.Vector2D;
import org.firstinspires.ftc.teamcode.util.Vector3D;

// Values recorded here are directly observed from sensors
public class Sensors {
    Telemetry telemetry;
    HardwareMap hardwareMap;

 //   private boolean initialized = false;

    private Vector3D orientation = new Vector3D(0, 0, 0); // TODO: LETS MOVE ALL DRIVETRAIN POSITION DATA TO DRIVETRAIN
    private Vector2D position2D = new Vector2D();

//    private Vector2D grabberPosition = new Vector2D(); // TODO: LETS MOVE ALL ARM POSITION DATA TO ARM
//    private Vector2D oldGrabberPosition = new Vector2D();
//    private Vector2D deltaGrabberPosition = new Vector2D();

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
    private DigitalChannel digitalChannel;

    public boolean lowerLimitA;
    public boolean lowerLimitB;
    public boolean baseLimit;
    public boolean tableLimit;

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

    private double time = 0;
    private double lastTime = 0;
    private double dt = 0; // the time interval since the last update function call

    public double getDt(){
        return dt;
    }
    public double getTime() { return time; }

    public CameraWrapper cameraWrapper;
    public Godrick godrick;

    public void initialize(Godrick godrick){
        this.godrick = godrick;
        telemetry = godrick.telemetry;
        hardwareMap = godrick.hardwareMap;
        cameraWrapper = new CameraWrapper(godrick.hardwareMap.appContext);

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
            digitalChannel = hardwareMap.get(DigitalChannel.class, "tableLimit");

            runtime.reset();

 //           initialized = true;
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

            // get the joint angles and angular velocities by reading the motor controller ticks
            StringBuilder sb = new StringBuilder();
            godrick.arm.turntable.setAngleByTicks(actuators.turnTable.getCurrentPosition());
            godrick.arm.turntable.setAngularVelocityByTicksPerSecond(actuators.turnTable.getVelocity());
            godrick.arm.baseJoint.setAngleByTicks(actuators.baseSegment.getCurrentPosition());
            godrick.arm.baseJoint.setAngularVelocityByTicksPerSecond(actuators.baseSegment.getVelocity());
            godrick.arm.elbowJoint.setAngleByTicks(actuators.lowerSegment.getCurrentPosition());
            godrick.arm.elbowJoint.setAngularVelocityByTicksPerSecond(actuators.lowerSegment.getVelocity());
            sb.append("th0 ");
            sb.append(godrick.arm.turntable.toString());
            sb.append("\tth1 ");
            sb.append(godrick.arm.baseJoint.toString());
            sb.append("\tth2 ");
            sb.append(godrick.arm.elbowJoint.toString());

            //Log.e("Sensors: update", sb.toString());

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

            lowerLimitA = touchLowerA.isPressed();
            lowerLimitB = touchLowerB.isPressed();
            baseLimit = touchBase.isPressed();
            tableLimit = digitalChannel.getState();

//            oldGrabberPosition = grabberPosition;
//            double grabberX = baseJointA.getX(UnitOfDistance.CM)+ elbowJoint.getX(UnitOfDistance.CM);
//            double grabberY = baseJointA.getY(UnitOfDistance.CM)+ elbowJoint.getY(UnitOfDistance.CM);
//            grabberPosition.set(grabberX, grabberY);
//            deltaGrabberPosition.set(grabberPosition.getX()-oldGrabberPosition.getX(), grabberPosition.getY()-oldGrabberPosition.getY());

            if(verbose) {
                telemetry.addData("Position: ", position.toString());
                telemetry.addData("Orientation: ", angles.toString());
                telemetry.addData("Turn: ", godrick.arm.turntable.getAngleDeg());
                telemetry.addData("BaseA: ", godrick.arm.baseJoint.getAngleDeg());
                telemetry.addData("Lower: ", godrick.arm.elbowJoint.getAngleDeg());

                telemetry.addData("Turn Target: ", godrick.arm.turntable.getTargetAngle());
                telemetry.addData("BaseA Target: ", godrick.arm.baseJoint.getTargetAngle());
                telemetry.addData("Lower Target: ", godrick.arm.elbowJoint.getTargetAngle());

                // buttons
                telemetry.addData("lower limit A: ", lowerLimitA);
                telemetry.addData("lower Limit B: ", lowerLimitB);
                telemetry.addData("base limit: ", baseLimit);
                telemetry.addData("table limit: ", tableLimit);

                telemetry.addData("Tolerance: ", godrick.actuators.lowerSegment.getTargetPositionTolerance());
            }
        }

        catch (Exception e) {
            telemetry.addData("Sensor update failure ", e.toString());
        }
    }

    Double formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    Double formatDegrees(double degrees){
        return AngleUnit.DEGREES.normalize(degrees);
    }
}
