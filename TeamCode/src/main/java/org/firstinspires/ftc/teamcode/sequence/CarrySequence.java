package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;

public class CarrySequence extends MotionSequence {
    public CarrySequence(ArmPose last) {
        ArmPose p = ArmPoseGenerator.rotateTo(0, last);
        sequence.add(p);
        sequence.add(ArmPoseGenerator.carryHigh(p));
    }
}
