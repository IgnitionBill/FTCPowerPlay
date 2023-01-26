package org.firstinspires.ftc.teamcode.sequence;

import android.util.Log;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.arm.ArmPose;
import org.firstinspires.ftc.teamcode.system.Godrick;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.UtilityKit;
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
    ArmPose currentPose;
    ArmController armController;

    public MotionSequenceDirector() {
        godrick = Godrick.getInstance();
        sensors = godrick.sensors;
        currentPose = godrick.sensors.currentPose;
        armController = godrick.armController;
    }

    public void update(){

        // if the current sequence is done, simply return
        if(currentSequence == null){
            return;
        }

        // if we reached the target position
        if(currentPose.closeTo(currentSequence.currentTarget())){
            // change to the next target, unless there is no next target, then the sequence is done
            if(!currentSequence.nextTarget()){
                currentSequence = null;
                return;
            }
        }

        // update the motion of the arm by moving further toward currentTarget
        armController.turnTableTicks = (int) (currentSequence.currentTarget().th0 * UtilityKit.ticksPerDegreeAtJoint);
        armController.baseTicks = (int) (currentSequence.currentTarget().th1 * UtilityKit.ticksPerDegreeAtJoint);
        armController.lowerTicks = (int) (currentSequence.currentTarget().th2 * UtilityKit.ticksPerDegreeAtJoint);
        armController.grabberRoll = currentSequence.currentTarget().th3;
        armController.grabberYaw = currentSequence.currentTarget().th4;
        armController.grabberPitch = currentSequence.currentTarget().th5;
        armController.grabberGrip = currentSequence.currentTarget().th6;
    }

    public void requestNewSequence(MotionSequenceName sequenceName){
        Log.e("MotionSequenceDirector", "requestNewSequence");
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
        Log.e("ArmController", "Trying to grab a cone");
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
        currentSequence = new CarryToPlaceToCarry(theCone);

    }

    private void goCarry(){
        currentSequence = new HomeToCarry();
    }

    private void goHome(){
        currentSequence = new CarryToHome();
    }
}
