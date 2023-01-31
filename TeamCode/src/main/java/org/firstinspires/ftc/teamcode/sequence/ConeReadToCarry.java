package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;

public class ConeReadToCarry extends MotionSequence{
    public ConeReadToCarry(){
        sequence.add(ArmPoseGenerator.coneRead);
        sequence.add(ArmPoseGenerator.carry);
    }
}
