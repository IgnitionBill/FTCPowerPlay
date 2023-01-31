package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;

public class CarryToConeRead extends MotionSequence{
    public CarryToConeRead(){
        sequence.add(ArmPoseGenerator.carry);
        sequence.add(ArmPoseGenerator.coneRead);
    }
}
