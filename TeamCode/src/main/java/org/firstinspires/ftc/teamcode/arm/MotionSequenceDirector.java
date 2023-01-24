package org.firstinspires.ftc.teamcode.arm;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.Vector3D;

public class MotionSequenceDirector {

    Sensors sensors;
    Telemetry telemetry;
    ArmPose currentPose;
    MotionSequence currentSequence;

    public MotionSequenceDirector(Sensors sensors, Telemetry telemetry) {
        this.sensors = sensors;
        this.telemetry = telemetry;
    }

    public void setCurrentPose(ArmPose currentPose) {
        this.currentPose = currentPose;
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
            }
        }
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
        // if the current sequence is not done, complete it
        else{

        }
    }

    //TODO: Add controller vibration to broadcast success/failure

    private void tryToGrab(){
        Log.e("ArmController", "Trying to grab a cone");
        // distance scan for a cone roughly straight ahead
        double[] position = sensors.cameraWrapper.scanForCone();
        // transform from camera coordinates to the robot coordinates
        Vector3D toCone = new Vector3D(position[0], position[1], position[2]);

        // if a target was not found, then give up
        if(toCone.z == 0 || toCone.z > .765){  // TODO: improve this test
            Log.e("MotionSequenceDirector", "Cone not found within reach");
            return;
        }

        CylindricalVector3D toConeCylindrical = new CylindricalVector3D();// = Robot.transformFromCameraToRobotCoordinates(toCone);

        Log.e("MotionSequenceDirector", "X " + toCone.x + " Y " + toCone.y + " Z " + toCone.z);
        Log.e("MotionSequenceDirector", " rho " + toConeCylindrical.rho + " th " + toConeCylindrical.theta + " z " + toConeCylindrical.z);

        // if a target was found, launch the motion sequence with the detected position
        currentSequence = new CarryToGrabToCarry(toConeCylindrical);

    }

    private void tryToPlace(){

    }

    private void goCarry(){

    }

    private void goHome(){

    }
}
