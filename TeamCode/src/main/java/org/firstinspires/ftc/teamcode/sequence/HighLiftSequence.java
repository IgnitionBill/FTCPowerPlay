package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;

public class HighLiftSequence extends MotionSequence{
    public HighLiftSequence(ArmPose last){
        sequence.add(ArmPoseGenerator.getGoingForHighLift(last));
        sequence.add(ArmPoseGenerator.up);
    }
}
