package org.firstinspires.ftc.teamcode.arm;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.Vector3D;

/**
 * aka the Armory
 */
public class ArmPoseGenerator {
    private static InverseKinematics inverseKinematics = new InverseKinematics();
    public static ArmPose home = new ArmPose(45.0, -85.0, 170.0, 45.0, 10.0, -90.0, 300.0);
    public static ArmPose carry = new ArmPose(0.0, -45.0, 135.0, 0.0, 0.0, 0.0, 300.0);
    public static ArmPose up = new ArmPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static ArmPose coneRead = new ArmPose(0.0, -10.0, 170.0, 0.0, 0.0, -90.0, 300.0);

    /**
     * This method will construct an ArmPose where the gripper is centered on the target.
     * The target position is specified in cylindrical, robot-centric coordinates (r, alpha, z)
     * @param target
     * @return
     */
//    public static ArmPose onTarget(Vector3D target){ // target is in robot coordinates
//        double th0, th1, th2, th3, th4, th5, th6;
//
//        th0 = target.theta; // from the bearing angle
//        th1 = target.theta; // from the law of cosines, and inverse kinematics
//        th2 = 0; // from the law of cosines, and inverse kinematics
//        th3 = 0; // straight wrist
//        th4 = 0;
//        th5 = 90 - th1 - th2; // for a level gripper, in degrees
//        th6 = 0; // future gripper
//        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
//    }

    public static ArmPose rotateTo(Vector3D target, ArmPose last) {
        CylindricalVector3D cylTarget = CylindricalVector3D.toCylindrical(target);

        double th0, th1, th2, th3, th4, th5, th6;
        th0 = cylTarget.theta; // angle to the target
        th1 = last.th1;
        th2 = last.th2;
        th3 = last.th3; // spin wrist
        th4 = last.th4; // for a level gripper, in degrees -- soon to change to wrist flex
        th5 = last.th5; // open gripper -- soon to change to side-to-side gripper (up down)
        th6 = last.th6; // future gripper
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }

    public static ArmPose gripAndMoveTo(Vector3D target) {
        CylindricalVector3D cylTarget = CylindricalVector3D.toCylindrical(target);
        Vector3D angles = InverseKinematics.calculateAngles(cylTarget);
        double th0, th1, th2, th3, th4, th5, th6;
        th0 = cylTarget.theta; // angle to the target
        th1 = angles.x; // from IK using (rho, z)
        th2 = angles.y;
        th3 = 0; // roll
        th4 = 0; // yaw
        th5 = angles.z; // for a level gripper, in degrees -- soon to change to wrist flex
        th6 = 300; // grip
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }

    public static ArmPose openGripAndMoveTo(Vector3D target) {
        CylindricalVector3D cylTarget = CylindricalVector3D.toCylindrical(target);
        Vector3D angles = InverseKinematics.calculateAngles(cylTarget);
        double th0, th1, th2, th3, th4, th5, th6;
        th0 = cylTarget.theta; // angle to the target
        th1 = angles.x; // from IK using (rho, z)
        th2 = angles.y;
        th3 = 0; // roll
        th4 = 0; // yaw
        th5 = angles.z; // for a level gripper, in degrees -- soon to change to wrist flex
        th6 = 0; // grip
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }
}
