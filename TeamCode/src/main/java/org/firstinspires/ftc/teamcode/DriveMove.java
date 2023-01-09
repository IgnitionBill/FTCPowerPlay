package org.firstinspires.ftc.teamcode;

public class DriveMove {
    // Gear Ratio Ratio = 19.2:1
    // Encoder Shaft: 28 pulses per revolution
    // Gearbox output: 537.7 pulses per revolution
    public int frontLeftTicks = 0;
    public int frontRightTicks = 0;
    public int backRightTicks = 0;
    public int backLeftTicks = 0;
    public String moveName = "Nameless";

    public DriveMove(int frontLeftTicks, int frontRightTicks, int backRightTicks, int backLeftTicks, String moveName) {
        this.frontLeftTicks = frontLeftTicks;
        this.frontRightTicks = frontRightTicks;
        this.backRightTicks = backRightTicks;
        this.backLeftTicks = backLeftTicks;
        this.moveName = moveName;
    }
}
