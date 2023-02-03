package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.Vector3D;

/**
 * aka the Armory
 */
public class ArmPoseGenerator {
    private static final int grabberOpen = 150;
    private static final int grabberClose = -150;

    private static InverseKinematics inverseKinematics = new InverseKinematics();
    public static ArmPose home = new ArmPose(Arm.TABLE_HOME, Arm.BASE_HOME, Arm.ELBOW_HOME, -45.0, -10.0, 90.0, grabberClose);
    public static ArmPose betweenHomeAndCarry = new ArmPose(0.0, 0.0, 120.0, 0.0, 0.0, -90.0, grabberClose);
    public static ArmPose carry = new ArmPose(0.0, -45.0, 120.0, 0.0, 0.0, 0.0, grabberClose);
    public static ArmPose up = new ArmPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, grabberOpen); // everything straight up
    public static ArmPose coneRead = new ArmPose(0.0, -10.0, 160.0, 0.0, 0.0, -90.0, grabberClose);
    public static ArmPose goingForHighLift = new ArmPose(0.0, -70.0, 90.0, 0.0, 0.0, 0.0, grabberClose);

    public static ArmPose carry(ArmPose last) {
        return new ArmPose(last.th0, -45.0, 135.0, 0.0, 0.0, 0.0, grabberClose);
    }

    public static ArmPose carryHigh(ArmPose last) {         // th5 moving poorly, cannot level even at 35 deg, should be 65
        return new ArmPose(last.th0, -65.0, 90.0, 0.0, 0.0, 35.0, grabberClose);
    }

    // joint 1 moves forward to assist joint 2 moving into place
    public static ArmPose betweenHomeAndCarry(ArmPose last){
        return new ArmPose(last.th0, 0.0, 135.0, 0.0, 0.0, 0.0, grabberClose);
    }

    public static ArmPose coneRead(ArmPose last) {
        return new ArmPose(last.th0, -10.0, 160.0, 0.0, 0.0, -90.0, grabberClose);
    }

    // joint 1 tilts back to assist joint 2 in the next phase
    public static ArmPose getGoingForHighLift(ArmPose last) {
        return new ArmPose(last.th0, -70.0, 90.0, 0.0, 0.0, 0.0, grabberClose);
    }

    public static ArmPose rotateTo(Vector3D target, ArmPose last) {
        CylindricalVector3D cylTarget = CylindricalVector3D.toCylindrical(target);

        double th0, th1, th2, th3, th4, th5, th6;
        th0 = cylTarget.theta + last.th0; // angle to the target
        th1 = last.th1;
        th2 = last.th2;
        th3 = last.th3; // spin wrist
        th4 = last.th4; // for a level gripper, in degrees -- soon to change to wrist flex
        th5 = last.th5; // open gripper -- soon to change to side-to-side gripper (up down)
        th6 = last.th6; // future gripper
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }

    public static ArmPose rotateTo(double angle, ArmPose last) {

        return new ArmPose(angle, last.th1, last.th2, last.th3, last.th4, last.th5, last.th6);
    }

    public static ArmPose gripAndMoveTo(Vector3D target, ArmPose last) {
        CylindricalVector3D cylTarget = CylindricalVector3D.toCylindrical(target);
        Vector3D angles = InverseKinematics.calculateAngles(cylTarget);
        double th0, th1, th2, th3, th4, th5, th6;
        th0 = cylTarget.theta + last.th0; // angle to the target
        th1 = angles.x; // from IK using (rho, z)
        th2 = angles.y;
        th3 = 0; // roll
        th4 = 0; // yaw
        th5 = angles.z; // for a level gripper, in degrees -- soon to change to wrist flex
        th6 = grabberClose; // grip
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }

    public static ArmPose openGripAndMoveTo(Vector3D target, ArmPose last) {
        CylindricalVector3D cylTarget = CylindricalVector3D.toCylindrical(target);
        Vector3D angles = InverseKinematics.calculateAngles(cylTarget);
        double th0, th1, th2, th3, th4, th5, th6;
        th0 = cylTarget.theta + last.th0; // angle to the target
        th1 = angles.x; // from IK using (rho, z)
        th2 = angles.y;
        th3 = 0; // roll
        th4 = 0; // yaw
        th5 = angles.z; // for a level gripper, in degrees -- soon to change to wrist flex
        th6 = grabberOpen; // grip
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }
}
