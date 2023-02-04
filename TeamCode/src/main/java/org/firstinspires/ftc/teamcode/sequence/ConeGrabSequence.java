package org.firstinspires.ftc.teamcode.sequence;

import android.util.Log;

import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;
import org.firstinspires.ftc.teamcode.navigation.Field;
import org.firstinspires.ftc.teamcode.util.Vector3D;

public class ConeGrabSequence extends MotionSequence {

    /**
     * This is the default grab sequence, aimed at 50.0 cm in front of the robot's facing
     */
    public ConeGrabSequence(ArmPose last){
        buildSequence(new Vector3D(60.0, 0.0, Field.CONE_HEIGHT_CM), last);
    }

    /**
     * This is the targeted grab sequence, that will attempt to pick up a cone at the target
     * position, in robot coordinates.
     * @param target
     */
    public ConeGrabSequence(Vector3D target, ArmPose last){
        buildSequence(target, last);
    }

    /**
     * This function generalizes the movement of grabbing a cone to a variable target position
     * that is already transformed to robot coordinates.
     * @param target
     */
    private void buildSequence(Vector3D target, ArmPose last){

        ArmPose getup = ArmPoseGenerator.carry(last);
        sequence.add(getup);

        // rotate to target
        ArmPose rotated = ArmPoseGenerator.rotateTo(target, getup);
        sequence.add(rotated);

        // move to just above the target and stop (later make it curve down smoothly if you want)
        Vector3D aboveTarget = new Vector3D(target.x, 0.0, target.z + Field.CONE_HEIGHT_CM); // y is zero because we already rotated to it
        Log.e("ConeGrabSequence: buildSequence", "targetVector: " + aboveTarget.toString());
        ArmPose above = ArmPoseGenerator.openGripAndMoveTo(aboveTarget, rotated);
        sequence.add(above);
//
//        // move down onto target
//        ArmPose onTarget = ArmPoseGenerator.openGripAndMoveTo(target, above);
//        sequence.add(onTarget);
//
//        // grip the cone until sensed
//        ArmPose gotIt = ArmPoseGenerator.gripAndMoveTo(target, onTarget);
//        sequence.add(gotIt);
//
//        // move upward
//       // sequence.add(ArmPoseGenerator.moveTo(aboveTarget));
//
//        // retract to carry position
//        sequence.add(ArmPoseGenerator.carryHigh(gotIt));
    }
}
