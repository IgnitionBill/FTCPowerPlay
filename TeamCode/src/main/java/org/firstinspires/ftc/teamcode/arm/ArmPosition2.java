package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.CoordinateCylinder;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;

public class ArmPosition2 {
    public double th0;
    public double th1;
    public double th2;
    public CoordinateCylinder position;
    private UnitOfAngle unitOfAngle;
    private UnitOfDistance unitOfDistance;

    ArmPosition2(double x, double y, Sensors sensors) {
        double distance = Math.sqrt(x*x+y*y);
        double length1 = sensors.baseData.getPositionDistance(UnitOfDistance.CM);
        double length2 = sensors.lowerData.getPositionDistance(UnitOfDistance.CM);
        double extraAngle = UtilityKit.atan(y/x, UnitOfAngle.DEGREES);

        th1 = 90 - extraAngle - UtilityKit.acos((length1*length1+distance*distance-length2*length2)/(2*length1*distance), UnitOfAngle.DEGREES);
        th2 = 180 - UtilityKit.acos((length1*length1+length2*length2-distance*distance)/(2*length1*length2), UnitOfAngle.DEGREES);
    }

    public double theta4ToBeLevel(){
        return 90 - th1 - th2;
    }
}
