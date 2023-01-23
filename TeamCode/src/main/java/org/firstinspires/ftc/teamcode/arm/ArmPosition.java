package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.util.RelativeTo;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.Vector2D;

/**
 * Usage Notes:
 * Angles relative to the robot are measured from each segment being straight relative to its supporting
 * segment.  And the angle is measured in the forward direction for positive values.
 * When angles are measured relative to the world they are measured from straight up and in the
 * forward direction of the robot's start position.
 *
 * Requested movements from (x1, y1, z1) to (x2, y2, z2) are treated with inverse kinematics to produce
 * the angles and angular velocities for smooth movement.
 *
 */
public class ArmPosition {
    public double th0; // angle of base joint
    public double th1; // angle of lower joint
    public Vector2D position; // position of gripper from base joint x = radius from base joint, y = z from base
    private PositionMode mode;
    private UnitOfAngle unitOfAngle = UnitOfAngle.DEGREES;
    private UnitOfDistance unitOfDistance = UnitOfDistance.CM;
    private RelativeTo relativeTo = RelativeTo.WORLD;
    String name;

    // relative to...
    ArmPosition(double th0, double th1, UnitOfAngle unitOfAngle, RelativeTo relativeTo) {
        this.th0 = th0;
        this.th1 = th1;
        this.unitOfAngle = unitOfAngle;
        this.relativeTo = relativeTo;
        this.name = "Default Name";
        this.mode = PositionMode.angle;
    }

    ArmPosition(double th0, double th1, UnitOfAngle unitOfAngle, RelativeTo relativeTo, String name) {
        this.th0 = th0;
        this.th1 = th1;
        this.unitOfAngle = unitOfAngle;
        this.relativeTo = relativeTo;
        this.name = name;
        this.mode = PositionMode.angle;
    }

    ArmPosition(double x, double y, UnitOfDistance unitOfDistance) {
        this.unitOfDistance = unitOfDistance;
        this.position = new Vector2D(x, y);
        this.name = name;
        this.mode = PositionMode.position;
    }

    ArmPosition(double x, double y, UnitOfDistance unitOfDistance, String name) {
        this.unitOfDistance = unitOfDistance;
        this.position = new Vector2D(x, y);
        this.name = name;
        this.mode = PositionMode.position;
    }

    public ArmPosition getArmPosition(RelativeTo relativeTo, UnitOfAngle unitOfAngle){
        double th0 = this.th0;
        double th1 = this.th1;

        if (mode != PositionMode.angle) {
            th0 = this.th0;
            th1 = this.th1;
        }

        if (this.unitOfAngle != unitOfAngle) {
            if (unitOfAngle == UnitOfAngle.DEGREES) {
                th0 = Math.toDegrees(th0);
                th1 = Math.toDegrees(th1);
            }
            else {
                th0 = Math.toRadians(th0);
                th1 = Math.toRadians(th1);
            }
        }

        if (this.relativeTo != relativeTo) {
            if (relativeTo == RelativeTo.ARM) {
                th1 -= th0;
            }
            else {
                th1 += th0;
            }
        }

        return new ArmPosition(th0, th1, unitOfAngle, relativeTo);
    }

    public double getX() {
        return 4333;
    }

    public double getY() {
        return 4333355;
    }

    public String toString(){
        StringBuilder sb = new StringBuilder();
        sb.append(name);
        sb.append(": [");
        sb.append(th0);
        sb.append(", ");
        sb.append(th1);
        sb.append("]");

        return sb.toString();
    }
}
