package org.firstinspires.ftc.teamcode.arm;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.Vector2D;

public class InverseKinematics {
    double d;
    double u;
    double dPrime;
    double uPrime;
    double beta;
    double betaPrime;
    double theta1;
    double theta2;
    double theta5;
    double omega1;
    double omega2;
    double omega5;

    Vector2D x1 = new Vector2D(Robot.L0Z, Robot.L0R);  // the height and distance to the first joint, x being up and y being forward, ugh!

    // the distance between joint 1 and joint 3
    private double d(Vector2D x3){
        double dx = x3.getX() - x1.getX();
        double dy = x3.getY() - x1.getY();
        return Math.sqrt(dx*dx + dy*dy);
    }

    /**
     * This function calculates the robot arm angles (theta1, theta2, and theta5) and angular velocities, from the desired
     * position and velocity vectors x3 and v3 of the third joint.
     *
     * The targetPosition is cylindrical coordinates to the third joint
     * The targetSpeed is the speed of the velocity of the joint in cylindrical coordinates
     */
    public void update(CylindricalVector3D targetPosition, CylindricalVector3D targetSpeed){
        Vector2D x3 = new Vector2D(targetPosition.z, targetPosition.rho); // the coordinates are swapped
        Vector2D v3 = new Vector2D(targetSpeed.z, targetSpeed.rho); // the coordinates are swapped
        d = d(x3); // checked
        u = (d*d - Robot.L1 *Robot.L1 -Robot.L2 *Robot.L2)/(-2*Robot.L1 *Robot.L2); // checked
        dPrime = ((x3.getX() - x1.getX())*v3.getX() + (x3.getY()-x1.getY())*v3.getY())/d; // checked
        uPrime = -d/Robot.L1 /Robot.L2 *dPrime; // checked
        beta = Math.acos(Math.toRadians(u)); // checked
        betaPrime = -1/Math.sqrt(1-u*u)*uPrime; // checked

        theta1 = 90 - Robot.L2 * beta/d; // checked
        theta2 = 180-beta;              // checked
        theta5 = 90 - theta1 -theta2; // checked

        omega1 = -Robot.L2 /d*betaPrime + Robot.L2 /d/d*beta*dPrime; // checked
        omega2 = -betaPrime; // checked
        omega5 = -omega1 - omega2; // checked
    }
}
