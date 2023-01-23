package org.firstinspires.ftc.teamcode.arm;


public class ArmPose {
    public double th0; // turn table
    public double th1; // base joint
    public double th2; // elbow joint
    public double th3; // wrist rotation
    public double th4; // wrist bend
    public double th5; // wrist flex
    public double th6; // later grip

    // rates of rotation for each joint
    public double om0;
    public double om1;
    public double om2;
    public double om3;
    public double om4;
    public double om5;
    public double om6;

    public final static double ANGULAR_THRESHOLD = 1; // one degree
    /*
    All arm position angles are measured relative to the standard robot coordinates.
    Forward is positive.
    Straight is 0 degrees.
    All angles are degrees.
     */
    public ArmPose(double th0, double th1, double th2, double th3, double th4, double th5, double th6){
        this.th0 = th0;
        this.th1 = th1;
        this.th2 = th2;
        this.th3 = th3;
        this.th4 = th4;
        this.th5 = th5;
        this.th6 = th6;
    }

    public void setOmegas(double om0, double om1, double om2, double om3, double om4, double om5, double om6){
        this.om0 = om0;
        this.om1 = om1;
        this.om2 = om2;
        this.om3 = om3;
        this.om4 = om4;
        this.om5 = om5;
        this.om6 = om6;
    }

    public boolean closeTo(ArmPose currentTarget) {
        if(Math.abs(currentTarget.th0 - th0) < ANGULAR_THRESHOLD){
            if(Math.abs(currentTarget.th1 - th1) < ANGULAR_THRESHOLD) {
                if (Math.abs(currentTarget.th2 - th2) < ANGULAR_THRESHOLD) {
                    if (Math.abs(currentTarget.th3 - th3) < ANGULAR_THRESHOLD) {
                        if (Math.abs(currentTarget.th4 - th4) < ANGULAR_THRESHOLD) {
                            if (Math.abs(currentTarget.th5 - th5) < ANGULAR_THRESHOLD) {
                                if (Math.abs(currentTarget.th6 - th6) < ANGULAR_THRESHOLD)
                                    return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    private boolean withinRange(double a, double b, double range){
        return a-b < range && b-a < range;
    }
}
