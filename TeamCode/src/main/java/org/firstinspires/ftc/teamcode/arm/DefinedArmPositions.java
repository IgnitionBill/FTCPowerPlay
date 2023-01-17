package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.util.RelativeTo;
import org.firstinspires.ftc.teamcode.util.UnitOfAngle;

/**
 * aka the Armory
 */
public class DefinedArmPositions {
    // Arm positions
    public ArmPosition foldedPosition; // = new ArmPosition(-60, 160, -160, 135, UnitOfAngle.DEGREES, RelativeTo.ARM, "Folded Position");
    public ArmPosition carryPosition; // = new ArmPosition(-30, 90, -90, 90, UnitOfAngle.DEGREES, RelativeTo.ARM, "Carry Position");
    public ArmPosition closePosition; // = new ArmPosition(-30, 90, 90, -105, UnitOfAngle.DEGREES, RelativeTo.ARM, "Close Grab Position");
    public ArmPosition farPosition; // = new ArmPosition(30, 15, 45, -90, UnitOfAngle.DEGREES, RelativeTo.ARM, "Far Grab Position");
    public ArmPosition highPosition; // = new ArmPosition(-60, 30, -100, -25, UnitOfAngle.DEGREES, RelativeTo.ARM, "High Junction Position");
    public ArmPosition middlePosition; // = new ArmPosition(-60, 30, 140, -65, UnitOfAngle.DEGREES, RelativeTo.ARM, "Middle Junction Position");
    public ArmPosition lowPosition; // = new ArmPosition(-60, 90, 130, -115, UnitOfAngle.DEGREES, RelativeTo.ARM, "Low Junction Position");
    public ArmPosition straightUpPosition; // = new ArmPosition(0, 0, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM, "Straight Up Position"); // 4 testing

    public ArmPosition placeTransition; // = new ArmPosition(0, 0, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM, "Transition");
    //    ArmPosition placeTransition = new ArmPosition(-20, 90, 90, -90, UnitOfAngle.DEGREES, RelativeTo.ARM, "Transition");
    public ArmPosition carryTransition; // = placeTransition; //new ArmPosition(-45, 120, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM);
    public ArmPosition grabTransition; // = placeTransition;
    public ArmPosition zeroPosition; // = new ArmPosition(0, 0, 0, 0, UnitOfAngle.DEGREES, RelativeTo.ARM, "Zero Position");
}
