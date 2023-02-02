package org.firstinspires.ftc.teamcode.sequence;

import android.util.Log;

import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.system.Godrick;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.Vector3D;

/**
 * The MotionSequenceDirector manages MotionSequences, which are playlists of motions.
 * A motion directs systems of the robot to perform some kind of action until some specified state
 * is achieved.  Motion sequences can be used in any op mode, to control any systems, however
 * in autonomous you want the drivetrain and arm under sequence control, while in driver mode, you
 * may want to control the drivetrain and deploy the arm motions as an automated sequence.  The
 * way to make it differently depends on the actions defined in the motion sequence.
 */
public class MotionSequenceDirector {

    Sensors sensors;
    MotionSequence currentSequence;
    Godrick godrick;
    ArmController armController;

    public MotionSequenceDirector() {

    }

    public void initialize(Godrick godrick){
        this.godrick = godrick;
        sensors = godrick.sensors;
        armController = godrick.armController;
    }

    public void update(){

        // if the current sequence is done, simply return
        if(currentSequence == null){
            return;
        }

        // if we reached the target position
        ArmPose currentPose = godrick.arm.getCurrentPose();
        if(currentPose.closeTo(currentSequence.currentTarget())){
            // change to the next target, unless there is no next target, then the sequence is done
            if(!currentSequence.nextTarget()){
                currentSequence = null;
                return;
            }
        }

        // update the motion of the arm by moving further toward currentTarget
        StringBuilder sb = new StringBuilder();
        sb.append(" Th0: " + currentSequence.currentTarget().th0);
        sb.append(" Th1: " + currentSequence.currentTarget().th1);
        sb.append(" Th2: " + currentSequence.currentTarget().th2);
        sb.append(" R: " + currentSequence.currentTarget().th3);
        sb.append(" Y: " + currentSequence.currentTarget().th4);
        sb.append(" P: " + currentSequence.currentTarget().th5);
        sb.append(" G: " + currentSequence.currentTarget().th6);
        Log.e("MotionSequenceDirector: update", "setting target: " + sb.toString());

        // Update the arm angles
        godrick.arm.turntable.setTargetAngle(currentSequence.currentTarget().th0);
        godrick.arm.baseJoint.setTargetAngle(currentSequence.currentTarget().th1);
        godrick.arm.elbowJoint.setTargetAngle(currentSequence.currentTarget().th2);
        godrick.arm.grabberRoll = currentSequence.currentTarget().th3;
        godrick.arm.grabberYaw = currentSequence.currentTarget().th4;
        godrick.arm.grabberPitch = currentSequence.currentTarget().th5;
        godrick.arm.grabberGrip = currentSequence.currentTarget().th6;
    }

    public void requestNewSequence(MotionSequenceName sequenceName){
        Log.e("MotionSequenceDirector", "requestNewSequence " + sequenceName.toString());
        // if the current sequence is done, try to perform the requested sequence
        if(currentSequence == null) {
            switch (sequenceName){
                case CarryToGrabToCarry:
                    tryToGrab();
                case CarryToPlaceToCarry:
                    tryToPlace();
                case HomeToCarry:
                    goCarry();
                case CarryToHome:
                    goHome();
            }
        }
    }

    //TODO: Add controller vibration to broadcast success/failure
    private void tryToGrab(){
        Log.e("ArmController", "Trying to grab a cone");
        // distance scan for a cone roughly straight ahead
        double[] position = sensors.cameraWrapper.scanForCone();
        if(position == null){
            Log.e("MotionSequenceDirector", "Null vector returned from camera wrapper.");
            return;
        }
        // transform from camera coordinates to the robot coordinates, meters to cm
        Vector3D toCone = new Vector3D(position[0], position[1], position[2]);

        // if a target was not found, then give up
        if(toCone.z == 0){
            Log.e("MotionSequenceDirector", "Cone not found");
            return;
        }

        // transform the vector from camera coordinates to robot coordinates
        Vector3D theCone = godrick.arm.transformCameraToRobotCoords(toCone);

        if(theCone.x > 76.5){
            Log.e("MotionSequenceDirector", "Cone too far");
            return;
        }

        if(theCone.x < 30){
            Log.e("MotionSequenceDirector", "Cone too close");
            return;
        }

        Log.e("MotionSequenceDirector", toCone.toString());
        Log.e("MotionSequenceDirector", theCone.toString());

        // create a motion sequence to grab the cone at the vector in robot coordinates
        currentSequence = new CarryToGrabToCarry(theCone);

    }

    private void tryToPlace(){
        Log.e("ArmController", "Trying to place a cone");
        // distance scan for a cone roughly straight ahead
        double[] position = sensors.cameraWrapper.scanForPole();
        if(position == null){
            Log.e("MotionSequenceDirector", "Null vector returned from camera wrapper.");
            return;
        }
        // transform from camera coordinates to the robot coordinates, meters to cm
        Vector3D toCone = new Vector3D(position[0], position[1], position[2]);

        // if a target was not found, then give up
        if(toCone.z == 0){
            Log.e("MotionSequenceDirector", "Pole not found");
            return;
        }

        // transform the vector from camera coordinates to robot coordinates
        Vector3D theCone = godrick.arm.transformCameraToRobotCoords(toCone);

        if(theCone.x > 76.5){
            Log.e("MotionSequenceDirector", "Pole too far");
            return;
        }

        if(theCone.x < 30){
            Log.e("MotionSequenceDirector", "Pole too close");
            return;
        }

        Log.e("MotionSequenceDirector", toCone.toString());
        Log.e("MotionSequenceDirector", theCone.toString());

        // create a motion sequence to grab the cone at the vector in robot coordinates
        currentSequence = new CarryToPlaceToCarry(theCone);

    }

    private void goCarry(){
        //
        Log.e("MotionSequenceDirector", "HomeToCarry");
        currentSequence = new HomeToCarry();
    }

    private void goHome(){
        Log.e("ArmController", "CarryToHome");
        currentSequence = new CarryToHome();
    }
}
