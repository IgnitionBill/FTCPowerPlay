package org.firstinspires.ftc.teamcode.arm;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CameraWrapper;
import org.firstinspires.ftc.teamcode.system.Sensors;
import org.firstinspires.ftc.teamcode.util.Vector3D;

public class MotionSequenceDirector {

    Sensors sensors;
    Telemetry telemetry;

    ArmPosition targetPosition; // target
    ArmPosition lastPosition; // previous position
    ArmPosition endPosition; // final position in sequence
    ArmPosition tickTarget; // ???
    ArmPosition currentPosition;
    private boolean currentSequenceDone = true;

    public MotionSequenceDirector(Sensors sensors, Telemetry telemetry) {
        this.sensors = sensors;
        this.telemetry = telemetry;
    }

    public void setCurrentPosition(ArmPosition currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void requestNewSequence(MotionSequenceName sequenceName){
        Log.e("MotionSequenceDirector", "requestNewSequence");
        // if the current sequence is done, try to perform the requested sequence
        if(currentSequenceDone) {
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
        Vector3D toCone = new Vector3D(position[0], position[1], position[2]);

        Log.e("Director Man says: ", "X: " + toCone.x + " Y: " + toCone.y + " Z: " + toCone.z);

        // if a target was not found, then give up

        // if a target was found, try to pick it up

        // launch the motion sequence with the scanned variables
    }

    private void tryToPlace(){

    }

    private void goCarry(){

    }

    private void goHome(){

    }
}
