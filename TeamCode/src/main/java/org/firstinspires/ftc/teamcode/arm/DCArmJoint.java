package org.firstinspires.ftc.teamcode.arm;

import android.util.Log;

import org.firstinspires.ftc.teamcode.util.UtilityKit;

public class DCArmJoint {
    private double angle; // joint angle in degrees
    private double maxRange; // the max angle of this joint in degrees, assumes it is properly zeroed
    private double minRange; // the min angle of this joint in degrees, assumes it is properly zeroed
    private double angularVelocity; // the angular velocity of the joint in degrees per second, as read from the motor encoders
    private double targetAngle; // the target angle in degrees
    private double targetAngularVelocity; // the target angular velocity
    private double homeAngle; // the angle offset in degrees where autohome regards as zero ticks

    public DCArmJoint(double minRange, double maxRange, double homeAngle){
        this.minRange = minRange;
        this.maxRange = maxRange;
        this.homeAngle = homeAngle;
        targetAngle = homeAngle;
    }

    // returns the current angle in degrees
    public double getAngleDeg(){
        return angle;
    }

    // get the angular velocity in degrees per second
    public double getAngularVelocityDegPS(){
        return angularVelocity;
    }

    // sets the angle using the number of ticks read from the motor encoder
    public void setAngleByTicks(int ticks){
        angle = UtilityKit.armTicksToDegrees(ticks) + homeAngle; // zero ticks corresponds to the home angle after autoHome
        angle = UtilityKit.limitToRange(angle, minRange, maxRange);
    }

    // convert ticks per second to degrees per second
    public void setAngularVelocityByTicksPerSecond(double tps){
        angularVelocity = UtilityKit.armTicksToDegrees(tps);
    }

    // sets the target angle in degrees
    public void setTargetAngle(double targetAngle){
        Log.e("DCArmJoint: setTargetAngle", "to: " + targetAngle);
        this.targetAngle = UtilityKit.limitToRange(targetAngle, minRange, maxRange);
    }

    // used for autohome, this does not limit the range of angle allowed because the angle is not set yet
    public void incrementTargetAngle(double increment){
        this.targetAngle += increment;
    }

    // set the target angular velocity in degrees per second
    public void setTargetAngularVelocity(double targetAngularVelocity){
        this.targetAngularVelocity = targetAngularVelocity;
    }

    // gets the target angle in ticks
    public int getTargetTicks(){
        return UtilityKit.armDegreesToTicks(targetAngle - homeAngle); // when target angle equals the home angle, we are at zero ticks after autoHome
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    // gets the target angular velocity in ticks per second
    public double getTargetTicksPerSecond(){
        return UtilityKit.armDegreesToTicks(targetAngularVelocity);
    }

    public String toString(){
        return "angle: " +  angle + " target: " + targetAngle;
    }

}
