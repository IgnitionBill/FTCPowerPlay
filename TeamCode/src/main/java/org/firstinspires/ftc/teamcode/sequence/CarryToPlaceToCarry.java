package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;
import org.firstinspires.ftc.teamcode.navigation.Field;
import org.firstinspires.ftc.teamcode.util.Vector3D;

public class CarryToPlaceToCarry extends MotionSequence {

    /**
     * This is the default grab sequence, aimed at 50.0 cm in front of the robot's facing
     */
    public CarryToPlaceToCarry(){
        buildSequence(new Vector3D(20.0, 0.0, Field.TALL_POLE_CM));
    }

    /**
     * This is the targeted grab sequence, that will attempt to pick up a cone at the target
     * position, in robot coordinates.
     * @param target
     */
    public CarryToPlaceToCarry(Vector3D target){
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

        // move to just above and in front of the target and stop (later make it curve down smoothly if you want)
        Vector3D beforeTarget = new Vector3D(target.x - Field.CONE_DIAMETER_CM, target.y, target.z + Field.CONE_HEIGHT_CM);
        sequence.add(ArmPoseGenerator.gripAndMoveTo(beforeTarget));

        // move to just above the target and stop (later make it curve down smoothly if you want)
        Vector3D aboveTarget = new Vector3D(target.x, target.y, target.z + Field.CONE_HEIGHT_CM);
        sequence.add(ArmPoseGenerator.gripAndMoveTo(aboveTarget));

        // move down onto target
        sequence.add(ArmPoseGenerator.gripAndMoveTo(target));

        // grip the cone until sensed
        sequence.add(ArmPoseGenerator.openGripAndMoveTo(aboveTarget));

        // move away
        sequence.add(ArmPoseGenerator.openGripAndMoveTo(beforeTarget));

        // retract to carry position
        sequence.add(ArmPoseGenerator.carry);
    }
}

