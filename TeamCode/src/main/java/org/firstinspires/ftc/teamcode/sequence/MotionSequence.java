package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPose;

import java.util.ArrayList;

/**
 * Motion sequence is like a playlist of ArmPoses
 */
public class MotionSequence {
    protected ArrayList<ArmPose> sequence = new ArrayList<>();
    private int current = 0; // the current target

    public MotionSequence(){

    }

    // get the current target ArmPose
    public ArmPose currentTarget(){
        return sequence.get(current);
    }

    public int getIndex() {
        return current;
    }

    // switch to the next target if there is one, otherwise switch back to start and return false
    public boolean nextTarget(){
        if(current+1 < sequence.size()){
            current++;
            return true;
        }
        current = 0; // start over
        return false;
    }
}
