package org.firstinspires.ftc.teamcode.arm;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.util.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.Matrix3x3;
import org.firstinspires.ftc.teamcode.util.Vector3D;

/**
 * Robot stats and data are stored here
 * Standard physical calculations of the robot are done here too
 * Robot coordinates are cylindrical coordinates specified relative to the center of the drivetrain.
 * The forward facing direction of the robot turntable will be the direction of r, as measured from the center
 * of the turntable.
 * The angle of the turntable facing relative to the field start position, will be the polar angle.
 * The height above the field will the z direction.
 */
public class Robot {
    double length; // length of bounding box of the robot drivetrain
    double width; // width of bounding box
    double height; // height of the bounding box of the drivetrain

    // length of each arm segment
    public final static double top = 10.2; // top of the turntable
    public final static double L0R = 12.5; // radial distance to the center of the first joint from the origin of robot:(12.5 cm, alpha, 23.0 cm)
    public final static double L0Z = 23.0; // height of the first joint above the floor
    public final static double L0 = Math.sqrt(L0R*L0R + L0Z*L0Z); // length from origin to first joint
    public final static double L1 = 34.0;  // cm, length of first segment
    public final static double L2 = 11.0; // cm, length of second segment
    public final static double L3 = 20.66; // because it rotates
    public final static double L4 = 2.84; // cm, to the center of the gripper?
    public final static double L5R = 8.0; // the forward displacement of the center of the gripper relative to the joint 5
    public final static double L5RTH = 9.0; // the rightward displacement to the center of the gripper relative to J5
    public final static double L34 = L3 + L4;
    public final static double LCAM = 6.2; // length from joint 5 to camera

    Vector3D position = new Vector3D(0, 0, 0);
    Vector3D velocity = new Vector3D(0,0,0);
    double theta = 0; // the robots drivetrain rotation
    double omega = 0; // the angular velocity of the drivetrain
    ArmPose currentPos = new ArmPose(0, 0, 0, 0, 0, 0, 0);

    // location of the turntable top center in robot coordinates
    CylindricalVector3D j0 = new CylindricalVector3D();

    // location of the center of the base joint in robot coordinates
    CylindricalVector3D j1 = new CylindricalVector3D();

    // location of the center of the elbow joint in robot coordinates
    CylindricalVector3D j2 = new CylindricalVector3D();

    // location of the center of the wrist joint in robot coordinates
    CylindricalVector3D j5 = new CylindricalVector3D();

    CylindricalVector3D cameraPosition = new CylindricalVector3D();

    CylindricalVector3D gripPosition = new CylindricalVector3D();

    // get the camera facing, a normal vector
    CylindricalVector3D cameraNormal = new CylindricalVector3D();

    // camera normal in cartesian coordinates
    Vector3D cameraNormalCart = new Vector3D();

    Vector3D l0 = new Vector3D();
    Vector3D l1 = new Vector3D();
    Vector3D l2 = new Vector3D();
    Vector3D lCam = new Vector3D();
    Vector3D lGrip = new Vector3D();

    Vector3D u0 = new Vector3D(0, 0, 1); // up
    Vector3D u1 = new Vector3D(0, 1, 0); // left
    Matrix3x3 r0 = new Matrix3x3();
    Matrix3x3 r1 = new Matrix3x3();
    
    public void update2(){ // x is forward, y is left, z is up
        // form the 0th segment
        l0.set(L0R, 0, L0Z);
        // set rotation about the unit vector u0 by an angle th0
        r0.setRotation(currentPos.th0, u0);
        // rotate the vector
        Vector3D p0 = r0.multiply(l0);
        // form the coordinate transformation using the vector and rotation, making the end of the segment the origin
        CoordinateSystem c0 = new CoordinateSystem(p0, r0);

        // form the first segment
        l1.set(0, 0, L1);
        // rotate about the unit vector u1 by an angle th1
        r1.setRotation(currentPos.th1, u1);
        // rotate the vector
        Vector3D p1a = r1.multiply(l1);
        // transform the vector by the previous transformation
        Vector3D p1 = c0.transformTo(p1a);
        // form the coordinate transformation
        CoordinateSystem c1 = new CoordinateSystem(p1, r1);

        // form the second segment
    }

    public void update(){
        // location of the turntable top center in robot coordinates
        j0.set(0.0, theta+currentPos.th0, top);
        // location of the center of the base joint in robot coordinates
        j1.set(L0R, j0.theta, L0Z);
        // location of the center of the elbow joint in robot coordinates
        j2.set(j1.rho + L1 * Math.sin(Math.toRadians(currentPos.th1)),
                j1.theta,
                j1.z + L1 * Math.cos(Math.toRadians(currentPos.th1)));
        // location of the center of the wrist joint in robot coordinates
        j5.set(j2.rho + L34 * Math.sin(Math.toRadians(currentPos.th2+currentPos.th1)),
                j2.theta,
                j2.z + L34 * Math.cos(Math.toRadians(currentPos.th2+currentPos.th1)));
        cameraPosition.set(j5.rho + LCAM * Math.sin(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)),
                j5.theta,
                j5.z + LCAM * Math.cos(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)));

        gripPosition.set(
                j5.rho + L5R * Math.sin(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)) ,
                j5.theta + L5RTH / gripPosition.rho,
                j5.z + L5R * Math.cos(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)));

        // this is the camera's normal vector
        cameraNormal.set(
                Math.sin(Math.toRadians(currentPos.th1 + currentPos.th2 + currentPos.th5)),
                j5.theta,
                Math.cos(Math.toRadians(currentPos.th1 + currentPos.th2 + currentPos.th5)));

        // camera normal in cartesian coordinates, centered on the robot
        cameraNormalCart = cameraNormal.toCartesian();

    }

    public CylindricalVector3D transformFromCameraToRobotCoordinates(Vector3D v){
        // transform the target vector v from camera coordinates to robot coordinates
        Vector3D vTransformed = new Vector3D(cameraNormalCart.x * v.z, cameraNormalCart.y, cameraNormalCart.z );

        // sum the camera vector with the target vector to get the target vector in robot coordinates
        return new CylindricalVector3D();
    }
}
