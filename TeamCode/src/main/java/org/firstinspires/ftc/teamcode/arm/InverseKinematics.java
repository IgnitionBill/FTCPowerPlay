package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.Vector2D;
import org.firstinspires.ftc.teamcode.util.Vector3D;

public class InverseKinematics {
    double d; // the distance from joint 1 to joint 3
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

    private static Vector2D x1 = new Vector2D(Arm.L0Z, Arm.L0R);  // the height and distance to the first joint, x being up and y being forward, ugh!

    // the distance between joint 1 and joint 3
    private static double d(Vector2D x3){
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
    public void update(Vector3D targetPosition, Vector3D targetSpeed){
        Vector2D x3 = new Vector2D(targetPosition.z, targetPosition.x); // the coordinates are swapped
        Vector2D v3 = new Vector2D(targetSpeed.z, targetSpeed.x); // the coordinates are swapped
        d = d(x3); // checked
        u = (d*d - Arm.L1 * Arm.L1 - Arm.L2 * Arm.L2)/(-2* Arm.L1 * Arm.L2); // checked
        dPrime = ((x3.getX() - x1.getX())*v3.getX() + (x3.getY()-x1.getY())*v3.getY())/d; // checked
        uPrime = -d/ Arm.L1 / Arm.L2 *dPrime; // checked
        beta = Math.acos(Math.toRadians(u)); // checked
        betaPrime = -1/Math.sqrt(1-u*u)*uPrime; // checked

        theta1 = 90 - Arm.L2 * beta/d; // checked
        theta2 = 180-beta;              // checked
        theta5 = 90 - theta1 -theta2; // checked

        omega1 = -Arm.L2 /d*betaPrime + Arm.L2 /d/d*beta*dPrime; // checked
        omega2 = -betaPrime; // checked
        omega5 = -omega1 - omega2; // checked
    }

    public static Vector3D calculateAngles(CylindricalVector3D target){
        Vector2D x3 = new Vector2D(target.z, target.rho); // the coordinates are swapped

        double d = d(x3); // checked
        double u = (d*d - Arm.L1 * Arm.L1 - Arm.L2 * Arm.L2)/(-2* Arm.L1 * Arm.L2); // checked
        double beta = Math.acos(Math.toRadians(u)); // checked

        double theta1 = 90 - Arm.L2 * beta/d; // checked
        double theta2 = 180-beta;              // checked
        double theta5 = 90 - theta1 -theta2; // checked

        return new Vector3D(theta1, theta2, theta5);
    }
}
