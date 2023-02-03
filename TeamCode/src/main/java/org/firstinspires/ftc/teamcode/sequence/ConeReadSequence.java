package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;

public class ConeReadSequence extends MotionSequence{
    public ConeReadSequence(ArmPose last){
        //sequence.add(ArmPoseGenerator.carry(last));
        sequence.add(ArmPoseGenerator.coneRead(last));
    }
}
