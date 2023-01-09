package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.RelativeTo;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;

public class ArmPosition {
    public double th0;
    public double th1;
    public double th2;
    public double th3;
    private UnitOfAngle unitOfAngle = UnitOfAngle.DEGREES;
    private RelativeTo relativeTo = RelativeTo.WORLD;
    String name;

    // relative to...
    ArmPosition(double th0, double th1, double th2, double th3, UnitOfAngle unitOfAngle, RelativeTo relativeTo) {
        this.th0 = th0;
        this.th1 = th1;
        this.th2 = th2;
        this.th3 = th3;
        this.unitOfAngle = unitOfAngle;
        this.relativeTo = relativeTo;
        this.name = "Default Name";
    }

    ArmPosition(double th0, double th1, double th2, double th3, UnitOfAngle unitOfAngle, RelativeTo relativeTo, String name) {
        this.th0 = th0;
        this.th1 = th1;
        this.th2 = th2;
        this.th3 = th3;
        this.unitOfAngle = unitOfAngle;
        this.relativeTo = relativeTo;
        this.name = name;
    }

    public ArmPosition getArmPosition(RelativeTo relativeTo, UnitOfAngle unitOfAngle){
        double th0 = this.th0;
        double th1 = this.th1;
        double th2 = this.th2;
        double th3 = this.th3;

        if (this.unitOfAngle != unitOfAngle) {
            if (unitOfAngle == UnitOfAngle.DEGREES) {
                th0 = Math.toDegrees(th0);
                th1 = Math.toDegrees(th1);
                th2 = Math.toDegrees(th2);
                th3 = Math.toDegrees(th3);
            }
            else {
                th0 = Math.toRadians(th0);
                th1 = Math.toRadians(th1);
                th2 = Math.toRadians(th2);
                th3 = Math.toRadians(th3);
            }
        }

        if (this.relativeTo != relativeTo) {
            if (relativeTo == RelativeTo.ARM) {
                th1 -= th0;
                th2 -= th1;
                th3 -= th2;
            }
            else {
                th1 += th0;
                th2 += th1;
                th3 += th2;
            }
        }

        return new ArmPosition(th0, th1, th2, th3, unitOfAngle, relativeTo);
    }

    public String toString(){
        StringBuilder sb = new StringBuilder();
        sb.append(name);
        sb.append(": [");
        sb.append(th0);
        sb.append(", ");
        sb.append(th1);
        sb.append(", ");
        sb.append(th2);
        sb.append(", ");
        sb.append(th3);
        sb.append("]");

        return sb.toString();
    }
}
