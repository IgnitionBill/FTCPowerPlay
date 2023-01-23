package org.firstinspires.ftc.teamcode.arm;

public class CarryToHome extends MotionSequence{
    public CarryToHome(){
        sequence.add(ArmPoseGenerator.carry);
        sequence.add(ArmPoseGenerator.home);
    }
}
