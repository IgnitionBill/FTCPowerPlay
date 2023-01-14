package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.RelativeTo;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;
import org.firstinspires.ftc.teamcode.util.UnitOfDistance;
import org.firstinspires.ftc.teamcode.util.UtilityKit;

/**
 * aka the Armory
 */
public class DefinedArmPositions {
    private Sensors sensors;

    public ArmPositionMod home = new ArmPositionMod(45, -85, 170, 45, 0, 0);
    public ArmPositionMod carry = new ArmPositionMod(0, -45, 135, 0, 0, 0);
   // public ArmPositionMod

    private final double heightAboveCone = 0; // 7 in above ground
    private final double lengthOfGripper = 0; // cm

    public DefinedArmPositions(Sensors sensors) {
        this.sensors = sensors;
    }

    public ArmPositionMod getGrabPosition(double radius, double turnTableAngle) {
    }
 }
