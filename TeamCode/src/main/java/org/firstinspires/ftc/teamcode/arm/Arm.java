package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.util.CoordinateTransformation;
import org.firstinspires.ftc.teamcode.util.Matrix3x3;
import org.firstinspires.ftc.teamcode.util.Vector3D;

/**
 * Robot arm stats and data are stored here
 * Standard physical calculations of the robot arm are done here too
 * Robot coordinates are relative to the center of the drivetrain at floor level.
 * Forward is x, left is y, up is z.
 * The angle of the turntable facing relative to the field start position, will be the polar angle.
 */
public class Arm {
    // lengths of each arm segment
    public final static double top = 10.2; // top of the turntable
    public final static double L0R = 12.5; // radial distance to the center of the first joint from the origin of robot:(12.5 cm, alpha, 23.0 cm)
    public final static double L0Z = 23.0; // height of the first joint above the floor
    public final static double L1 = 34.0;  // cm, length of first segment
    public final static double L2 = 11.0; // cm, length of second segment
    public final static double L2X = -2.4;
    public final static double L3 = 20.66; // cm, length of the third segment
    public final static double L4 = 2.84; // cm, length of the fourth segment
    public final static double LCAM = 6.2; // length from joint 5 to camera face
    public final static double LGRIPZ = 8.0; // forward length from joint 5 to the center of the gripper
    public final static double LGRIPX = -9.0; // leftward distance from joint 5 to the center of the gripper
    public final static double TABLE_HOME = 45;
    public final static double BASE_HOME = -74;
    public final static double ELBOW_HOME = 165;

    // ARM JOINTS RUN BY DC MOTORS
    public DCArmJoint turntable = new DCArmJoint(-180, 180, TABLE_HOME);
    public DCArmJoint baseJoint = new DCArmJoint(-74, 90, BASE_HOME);
    public DCArmJoint elbowJoint = new DCArmJoint(0, 165, ELBOW_HOME);

    public double grabberRoll = 0; // roll
    public double grabberPitch = 0; // pitch
    public double grabberYaw = 0; // yaw
    public double grabberGrip = 0; // grip

    // The angles of each arm joint as measured from straight being 0 degrees
    private ArmPose currentPos = new ArmPose(0, 0, 0, 0, 0, 0, 0);

    // Arm segments represented as vectors
    Vector3D l0 = new Vector3D(L0R, 0, L0Z);
    Vector3D l1 = new Vector3D(L2X, 0, L1);
    Vector3D l2 = new Vector3D(-L2X, 0, L2);
    Vector3D l3 = new Vector3D(0, 0, L3);
    Vector3D l4 = new Vector3D(0, 0, L4);
    Vector3D lCam = new Vector3D(0, 0, LCAM);
    Vector3D lGrip = new Vector3D(LGRIPX, 0, LGRIPZ);

    public double maxArmHeight = l0.z+l1.z+l2.z;

    // unit vector defining an axis of rotation
    Vector3D u0 = new Vector3D(0, 0, 1); // up
    Vector3D u1 = new Vector3D(0, 1, 0); // left
    Vector3D u2 = new Vector3D(0, 1, 0); // left
    Vector3D u3 = new Vector3D(1, 0, 0); // forward
    Vector3D u4 = new Vector3D(0, 0, 1); // up
    Vector3D u5 = new Vector3D(0, 1, 0); // left

    // rotation matrix of each joint
    Matrix3x3 r0 = new Matrix3x3();
    Matrix3x3 r1 = new Matrix3x3();
    Matrix3x3 r2 = new Matrix3x3();
    Matrix3x3 r3 = new Matrix3x3();
    Matrix3x3 r4 = new Matrix3x3();
    Matrix3x3 r5 = new Matrix3x3();

    // the center of each joint in robot coordinates
    Vector3D p0;
    Vector3D p1;
    Vector3D p2;
    Vector3D p3;
    Vector3D p4;
    Vector3D pGrip;
    Vector3D pCam;

    // the coordinate transformation for each arm segment
    CoordinateTransformation c0;
    CoordinateTransformation c1;
    CoordinateTransformation c2;
    CoordinateTransformation c3;
    CoordinateTransformation c4;
    CoordinateTransformation cCam;
    CoordinateTransformation cGrip;

    // combined coordinate transformations that take you from the robot to the camera or gripper... or back
    CoordinateTransformation robotToCamera = new CoordinateTransformation();
    CoordinateTransformation robotToGripper = new CoordinateTransformation();

    public ArmPose getCurrentPose(){
        return new ArmPose(
                turntable.getAngleDeg(),
                baseJoint.getAngleDeg(),
                elbowJoint.getAngleDeg(),
                grabberRoll,
                grabberYaw,
                grabberPitch,
                grabberGrip
                );
    }

    /**
     * The kinematic update takes motor encoder- or rotary encoder-read joint angles and
     * Robot parameters to calculate the positions and orientation of each arm segment,
     * building CoordinateTransformations from the base of the robot to the camera and grip center.
     */
    public void kinematicUpdate(){
        // form the 0th segment
        // set rotation about the unit vector u0 by an angle th0
        r0.setRotation(currentPos.th0, u0);
        // rotate the vector
        Vector3D p0 = r0.multiply(l0);
        // form the coordinate transformation using the vector and rotation, making the end of the segment the origin
        CoordinateTransformation c0 = new CoordinateTransformation(p0, r0);

        // form the first segment
        // rotate about the unit vector u1 by an angle th1
        r1.setRotation(currentPos.th1, u1);
        // rotate the vector
        Vector3D p1a = r1.multiply(l1);
        // transform the vector by the previous transformation
        p1 = c0.transform(p1a);
        // form the coordinate transformation
        c1 = new CoordinateTransformation(p1, r1);

        // form the second segment
        // rotate about the unit vector u2 by an angle th2
        r2.setRotation(currentPos.th2, u2);
        // rotate the vector
        Vector3D p2a = r2.multiply(l2);
        // transform the vector by the previous transformations
        Vector3D p2b = c0.transform(p2a);
        p2 = c1.transform(p2b);
        // form the coordinate transformation
        c2 = new CoordinateTransformation(p2, r2);

        // form the third segment
        // rotate about the unit vector u3 by an angle th3
        r3.setRotation(currentPos.th3, u3);
        // rotate the vector
        Vector3D p3a = r3.multiply(l3);
        // transform the vector by the previous transformations
        Vector3D p3b = c0.transform(p3a);
        Vector3D p3c = c1.transform(p3b);
        p3 = c2.transform(p3c);
        // form the coordinate transformation
        c3 = new CoordinateTransformation(p3, r3);

        // form the fourth segment
        // rotate about the unit vector u3 by an angle th3
        r4.setRotation(currentPos.th4, u4);
        // rotate the vector
        Vector3D p4a = r4.multiply(l4);
        // transform the vector by the previous transformations
        Vector3D p4b = c0.transform(p4a);
        Vector3D p4c = c1.transform(p4b);
        Vector3D p4d = c2.transform(p4c);
        p4 = c3.transform(p4d);
        // form the coordinate transformation
        c4 = new CoordinateTransformation(p4, r4);

        // form the fifth, or gripper, segment
        // rotate about the unit vector u5 by an angle th5
        r5.setRotation(currentPos.th5, u5);
        // rotate the grip vector
        Vector3D pGripR = r5.multiply(lGrip);
        // transform the vector by the previous transformations
        Vector3D pGrip0 = c0.transform(pGripR);
        Vector3D pGrip1 = c1.transform(pGrip0);
        Vector3D pGrip2 = c2.transform(pGrip1);
        Vector3D pGrip3 = c3.transform(pGrip2);
        pGrip = c4.transform(pGrip3);

        Vector3D pCamR = r5.multiply(lCam);
        Vector3D pCam0 = c0.transform(pCamR);
        Vector3D pCam1 = c1.transform(pCam0);
        Vector3D pCam2 = c2.transform(pCam1);
        Vector3D pCam3 = c3.transform(pCam2);
        pCam = c4.transform(pCam3);
        
        // form the coordinate transformation
        cCam = new CoordinateTransformation(pCam, r5);
        cGrip = new CoordinateTransformation(pGrip, r5.copy());

        // combined translations
        Vector3D joint5 = p0.add(p1).add(p2).add(p3).add(p4);
        Vector3D cameraEye = joint5.add(pCam); // this should be the same as the origin in robotToCamera below
        Vector3D grip = joint5.add(pGrip); // this should be the same as the origin in the robotToGripper below

        // combined transformations
        CoordinateTransformation t = c0.mergeToCombine(c1).mergeToCombine(c2).mergeToCombine(c3)
                .mergeToCombine(c4);
        robotToCamera = t.mergeToCombine(cCam);
        robotToGripper = t.mergeToCombine(cGrip);
    }

    public static void test(){
        Arm arm = new Arm(); // can be a singleton
        arm.currentPos.setThetas(0, 0, 0, 0, 0, 0, 0);
        arm.kinematicUpdate();
        // check all numbers
    }

    public Vector3D transformCameraToRobotCoords(Vector3D toCone) {
        Vector3D mixedUp = new Vector3D(toCone.z, -toCone.x, -toCone.y);
        return robotToCamera.inverseTransform(mixedUp); // TODO: IS THIS SWITCHING FROM Z OUT TO Z UP?
    }
}
