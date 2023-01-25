package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;
import org.firstinspires.ftc.teamcode.sequence.MotionSequence;

public class HomeToCarry extends MotionSequence {
    public HomeToCarry() {
        sequence.add(ArmPoseGenerator.home);
        sequence.add(ArmPoseGenerator.carry);
    }
}
