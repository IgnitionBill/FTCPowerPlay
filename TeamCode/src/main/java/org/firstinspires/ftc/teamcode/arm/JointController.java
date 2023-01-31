package org.firstinspires.ftc.teamcode.arm;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class JointController {
    public String deviceName;
    public DcMotorEx dcMotor;
    private boolean atTarget;

    public JointController(HardwareMap hardwareMap, String deviceName) {

        try {
            this.deviceName = deviceName;
            dcMotor = hardwareMap.get(DcMotorEx.class, deviceName);
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        catch (Exception e){
            Log.e("JointController", e.toString());
        }

        //dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset() {dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void setTarget(int ticks) {dcMotor.setTargetPosition(ticks);}
    public void  setMode() {dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public void setPower(double power) {dcMotor.setPower(power);}
    public void setVelocity(double ticksPS) {dcMotor.setVelocity(ticksPS);} // in ticks per second
    public boolean isAtTarget() {return atTarget;}
    public double getVelocity() { // Returns in ticks
        return dcMotor.getVelocity();
    } // in ticks per second
    public int getCurrentPosition() { // Returns in ticks
        return dcMotor.getCurrentPosition();
    }
    public void reverse() {dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);}
}
