package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;
import org.firstinspires.ftc.teamcode.sequence.MotionSequence;

public class CarryToHome extends MotionSequence {
    public CarryToHome(){
        //sequence.add(ArmPoseGenerator.carry);
        sequence.add(ArmPoseGenerator.home);
    }
}
