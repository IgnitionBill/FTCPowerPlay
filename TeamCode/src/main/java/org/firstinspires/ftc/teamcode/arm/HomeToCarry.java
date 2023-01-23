package org.firstinspires.ftc.teamcode.arm;

public class HomeToCarry extends MotionSequence{
    public HomeToCarry() {
        sequence.add(ArmPoseGenerator.home);
        sequence.add(ArmPoseGenerator.carry);
    }
}
