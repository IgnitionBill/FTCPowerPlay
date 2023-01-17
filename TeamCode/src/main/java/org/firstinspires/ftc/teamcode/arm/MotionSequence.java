package org.firstinspires.ftc.teamcode.arm;

import java.util.ArrayList;

public class MotionSequence {
    ArrayList<ArmPosition> sequence = new ArrayList<>();
    MotionSequenceName name;

    public MotionSequence(MotionSequenceName name){
        this.name = name;
    }
    public void add(ArmPosition armPosition){
        sequence.add(armPosition);
    }
}
