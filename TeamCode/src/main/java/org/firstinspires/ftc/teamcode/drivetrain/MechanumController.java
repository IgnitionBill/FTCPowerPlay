package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.teamcode.drivetrain.DriveMove;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;
import org.firstinspires.ftc.teamcode.util.Vector2D;

public class MechanumController {
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
}
