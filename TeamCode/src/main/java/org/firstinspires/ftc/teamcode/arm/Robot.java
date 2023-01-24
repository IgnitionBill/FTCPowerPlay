package org.firstinspires.ftc.teamcode.arm;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
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
    public final static double L1 = 34.0;  // cm
    public final static double L2 = 11.0; // cm
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
    CylindricalVector3D j0 = new CylindricalVector3D(0.0, theta+currentPos.th0, top);

    // location of the center of the base joint in robot coordinates
    CylindricalVector3D j1 = new CylindricalVector3D(L0R, j0.theta, L0Z);

    // location of the center of the elbow joint in robot coordinates
    CylindricalVector3D j2 = new CylindricalVector3D(j1.rho + L1 * Math.sin(Math.toRadians(currentPos.th1)),
            j1.theta,
            j1.z + L1 * Math.cos(Math.toRadians(currentPos.th1)));

    // location of the center of the wrist joint in robot coordinates
    CylindricalVector3D j5 = new CylindricalVector3D(j2.rho + L34 * Math.sin(Math.toRadians(currentPos.th2+currentPos.th1)),
            j2.theta,
            j2.z + L34 * Math.cos(Math.toRadians(currentPos.th2+currentPos.th1)));

    CylindricalVector3D cameraPosition = new CylindricalVector3D(
            j5.rho + LCAM * Math.sin(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)) ,
            j5.theta,
            j5.z + LCAM * Math.cos(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)));

    CylindricalVector3D gripPosition = new CylindricalVector3D(
            j5.rho + L5R * Math.sin(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)) ,
            j5.theta, // this one is modified later, see below
            j5.z + L5R * Math.cos(Math.toRadians(currentPos.th5+currentPos.th2+currentPos.th1)));

    // get the camera facing, a normal vector
    CylindricalVector3D cameraNormal = new CylindricalVector3D(
            Math.sin(Math.toRadians(currentPos.th1 + currentPos.th2 + currentPos.th5)),
            0,
            Math.cos(Math.toRadians(currentPos.th1 + currentPos.th2 + currentPos.th5)));

    // camera normal in cartesian coordinates
    Vector3D cameraNormalCart = new Vector3D();

    CylindricalVector3D l0 = new CylindricalVector3D();
    CylindricalVector3D l1 = new CylindricalVector3D();
    CylindricalVector3D l2 = new CylindricalVector3D();
    CylindricalVector3D lCam = new CylindricalVector3D();
    CylindricalVector3D lGrip = new CylindricalVector3D();

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

    }
}
