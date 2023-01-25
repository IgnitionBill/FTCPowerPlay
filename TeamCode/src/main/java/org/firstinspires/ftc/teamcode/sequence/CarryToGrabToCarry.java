package org.firstinspires.ftc.teamcode.sequence;

import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;
import org.firstinspires.ftc.teamcode.navigation.Field;
import org.firstinspires.ftc.teamcode.sequence.MotionSequence;
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;

public class CarryToGrabToCarry extends MotionSequence {

    /**
     * This is the default grab sequence, aimed at 50.0 cm in front of the robot's facing
     */
    public CarryToGrabToCarry(){
        buildSequence(new CylindricalVector3D(50.0, 0.0, 0.0));
    }

    /**
     * This is the targeted grab sequence, that will attempt to pick up a cone at the target
     * position, in robot coordinates.
     * @param target
     */
    public CarryToGrabToCarry(CylindricalVector3D target){
        buildSequence(target);
    }

    /**
     * This function generalizes the movement of grabbing a cone to a variable target position
     * that is already transformed to robot coordinates.
     * @param target
     */
    private void buildSequence(CylindricalVector3D target){
        // start at carry
        sequence.add(ArmPoseGenerator.carry);

        // rotate to target
        sequence.add(ArmPoseGenerator.rotateTo(target));

        // move to just above the target and stop (later make it curve down smoothly if you want)
        CylindricalVector3D aboveTarget = new CylindricalVector3D(target.rho, target.theta, target.z + Field.CONE_HEIGHT_CM);
        sequence.add(ArmPoseGenerator.releaseGripAndMoveTo(aboveTarget));

        // move down onto target
        sequence.add(ArmPoseGenerator.releaseGripAndMoveTo(target));

        // grip the cone until sensed
        sequence.add(ArmPoseGenerator.gripAndMoveTo(target));

        // move upward
       // sequence.add(ArmPoseGenerator.moveTo(aboveTarget));

        // retract to carry position
        sequence.add(ArmPoseGenerator.carry);
    }
}
