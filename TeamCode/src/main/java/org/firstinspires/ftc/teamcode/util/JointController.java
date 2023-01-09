package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class JointController {
    public String deviceName;
    public DcMotorEx dcMotor;
    private boolean atTarget;

    public JointController(HardwareMap hardwareMap, String deviceName) {
        this.deviceName = deviceName;
        dcMotor = hardwareMap.get(DcMotorEx.class, deviceName);
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTarget(int ticks) {dcMotor.setTargetPosition(ticks);}
    public void setPower(double power) {dcMotor.setPower(power);}
    public void setVelocity(double ticksPS) {dcMotor.setVelocity(ticksPS);}
    public boolean isAtTarget() {return atTarget;}
    public double getVelocity() { // Returns in ticks
        return dcMotor.getVelocity();
    }
    public int getCurrentPosition() { // Returns in ticks
        return dcMotor.getCurrentPosition();
    }
}
