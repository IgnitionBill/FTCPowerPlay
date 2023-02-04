package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.GamePadState;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;
import org.firstinspires.ftc.teamcode.util.Vector2D;

public class MechanumController {
    // Gear Ratio Ratio = 19.2:1
    // Encoder Shaft: 28 pulses per revolution
    // Gearbox output: 537.7 pulses per revolution
    public int frontLeftTicks = 0;
    public int frontRightTicks = 0;
    public int backRightTicks = 0;
    public int backLeftTicks = 0;

    public double frontLeft;
    public double frontRight;
    public double backRight;
    public double backLeft;

    public final static int BASE_TICKS = 100;

    Telemetry telemetry;

    public void initialize(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public DriveMove moveInDirection(double distance, UnitOfDistance unitOfDistance, double angle, UnitOfAngle unitOfAngle, String moveName) {
        //TODO make unit conversion functions
        if (unitOfDistance != UnitOfDistance.IN) {
            return null;
        }
        else{

        }
        if (unitOfAngle != UnitOfAngle.DEGREES) {
            return null;
        }

        Vector2D target = new Vector2D(UtilityKit.sin(angle, UnitOfAngle.DEGREES)*distance, UtilityKit.cos(angle, UnitOfAngle.DEGREES)*distance);

        double frontLeft = target.getY()+target.getX();
        double frontRight = target.getY()-target.getX();
        double backRight = target.getY()+target.getX();
        double backLeft = target.getY()-target.getX();

        int frontLeftTicks = UtilityKit.driveDistanceToTicks(frontLeft, UnitOfDistance.IN);
        int frontRightTicks = UtilityKit.driveDistanceToTicks(frontRight, UnitOfDistance.IN);
        int backRightTicks = UtilityKit.driveDistanceToTicks(backRight, UnitOfDistance.IN);
        int backLeftTicks = UtilityKit.driveDistanceToTicks(backLeft, UnitOfDistance.IN);

        return new DriveMove(frontLeftTicks, frontRightTicks, backRightTicks, backLeftTicks, moveName);
    }

    public void simpleMechanumUpdate(GamePadState gamePadState, Sensors sensorState, boolean verbose){

        double xL = flatten(gamePadState.leftStickX);
        double yL = flatten(gamePadState.leftStickY);
        double xR = flatten(gamePadState.rightStickX);
        double yR = flatten(gamePadState.rightStickY);

        // get the magnitude of the stick deflection
        double magnitude = Math.sqrt(xL*xL + yL*yL + xR*xR);

        // add the vector components, rescaled to a maximum of the magnitude given
        if (magnitude > .9) {
            frontLeft = -(-yL + xL + xR);//magnitude;
            frontRight = (-yL - xL - xR);//magnitude;
            backRight = (-yL + xL - xR);//magnitude;
            backLeft = -(-yL - xL + xR);//magnitude;
        }
        else if(magnitude > .001) {
            frontLeft = -(-yL + xL + xR)/2;//magnitude;
            frontRight = (-yL - xL - xR)/2;//magnitude;
            backRight = (-yL + xL - xR)/2;//magnitude;
            backLeft = -(-yL - xL + xR)/2;//magnitude;
        }
        else{
            frontLeft = 0.0;
            frontRight = 0.0;
            backRight = 0.0;
            backLeft = 0.0;
        }

        if (frontLeft > 1) { frontLeft = 1; }
        else if (frontLeft < -1) { frontLeft = -1; }

        if (frontRight > 1) { frontRight = 1; }
        else if (frontRight < -1) { frontRight = -1; }

        if (backRight > 1) { backRight = 1; }
        else if (backRight < -1) { backRight = -1; }

        if (backLeft > 1) { backLeft = 1; }
        else if (backLeft < -1) { backLeft = -1; }

        frontLeftTicks = sensorState.oldFrontLeftPosition + (int)(BASE_TICKS*frontLeft);
        frontRightTicks = sensorState.oldFrontRightPosition + (int)(BASE_TICKS*frontRight);
        backRightTicks = sensorState.oldBackRightPosition + (int)(BASE_TICKS*backRight);
        backLeftTicks = sensorState.oldBackLeftPosition + (int)(BASE_TICKS*backLeft);

        if(verbose){
            telemetry.addData("frontLeftTicks: ", frontLeftTicks);
            telemetry.addData("frontRightTicks: ", frontRightTicks);
            telemetry.addData("backRightTicks: ", backRightTicks);
            telemetry.addData("backLeftTicks: ", backLeftTicks);
        }
    }

    // limits the range of the value to +1 to -1 and squares it, preserving the sign
    private double flatten(double value){
        if(value > 1.0){
            value = 1.0;
        }
        else if(value < -1.0){
            value = -1.0;
        }
        return value * Math.abs(value);
    }
}
