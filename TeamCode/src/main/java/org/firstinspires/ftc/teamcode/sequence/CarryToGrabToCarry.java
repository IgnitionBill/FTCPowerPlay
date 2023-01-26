package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;
import org.firstinspires.ftc.teamcode.navigation.Field;
import org.firstinspires.ftc.teamcode.util.Vector3D;

public class CarryToGrabToCarry extends MotionSequence {

    /**
     * This is the default grab sequence, aimed at 50.0 cm in front of the robot's facing
     */
    public CarryToGrabToCarry(){
        buildSequence(new Vector3D(50.0, 0.0, 0.0));
    }

    /**
     * This is the targeted grab sequence, that will attempt to pick up a cone at the target
     * position, in robot coordinates.
     * @param target
     */
    public CarryToGrabToCarry(Vector3D target){
        buildSequence(target);
    }

    /**
     * This function generalizes the movement of grabbing a cone to a variable target position
     * that is already transformed to robot coordinates.
     * @param target
     */
    private void buildSequence(Vector3D target){
        // start at carry
        sequence.add(ArmPoseGenerator.carry); // TODO: DO WE NEED THIS?

        // rotate to target
        sequence.add(ArmPoseGenerator.rotateTo(target, ArmPoseGenerator.carry));

        // move to just above the target and stop (later make it curve down smoothly if you want)
        Vector3D aboveTarget = new Vector3D(target.x, target.y, target.z + Field.CONE_HEIGHT_CM);
        sequence.add(ArmPoseGenerator.openGripAndMoveTo(aboveTarget));

        // move down onto target
        sequence.add(ArmPoseGenerator.openGripAndMoveTo(target));

        // grip the cone until sensed
        sequence.add(ArmPoseGenerator.gripAndMoveTo(target));

        // move upward
       // sequence.add(ArmPoseGenerator.moveTo(aboveTarget));

        // retract to carry position
        sequence.add(ArmPoseGenerator.carry);
    }
}
