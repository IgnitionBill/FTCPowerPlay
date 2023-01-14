package org.firstinspires.ftc.teamcode.arm;

import java.util.ArrayList;

public class DefinedMotionSequences {

    // Arm sequences
    public MotionSequence homeToCarry = new MotionSequence(MotionSequenceName.HomeToCarry);
    public MotionSequence carryToHome = new MotionSequence(MotionSequenceName.CarryToHome);
    public MotionSequence carryToGrab = new MotionSequence(MotionSequenceName.CarryToGrabToCarry);
    public MotionSequence carryToPlace = new MotionSequence(MotionSequenceName.CarryToPlaceToCarry);

    public void init(DefinedArmPositions definedArmPositions){

//        homeToCarry.add(definedArmPositions.carryPosition);

        //////////////////////////////////////////////////////////////////////////////////////////

 //       carryToHome.add(definedArmPositions.home);

        //////////////////////////////////////////////////////////////////////////////////////////

 //       carryToGrab.add(definedArmPositions.g);
    }
}
