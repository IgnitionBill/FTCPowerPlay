package org.firstinspires.ftc.teamcode.arm;

public class ArmPositionMod {
    public double th0; // turn table
    public double th1; // base joint
    public double th2; // elbow joint
    public double th3; // wrist rotation
    public double th4; // wrist bend
    public double th5; // grip

    /*
    All arm position angles are measured relative to the standard robot coordinates.
    Forward is positive.
    Straight is 0 degrees.
    All angles are degrees.
     */
    public ArmPositionMod(double th0, double th1, double th2, double th3, double th4, double th5){
        this.th0 = th0;
        this.th1 = th1;
        this.th2 = th2;
        this.th3 = th3;
        this.th4 = th4;
        this.th5 = th5;
    }
}
