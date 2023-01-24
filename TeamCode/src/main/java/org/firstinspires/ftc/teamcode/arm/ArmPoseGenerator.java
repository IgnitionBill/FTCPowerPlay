package org.firstinspires.ftc.teamcode.arm;

import org.firstinspires.ftc.teamcode.util.CylindricalVector3D;
import org.firstinspires.ftc.teamcode.util.Vector2D;
import org.firstinspires.ftc.teamcode.util.Vector3D;

/**
 * aka the Armory
 */
public class ArmPoseGenerator {
    private static InverseKinematics inverseKinematics = new InverseKinematics();
    public static ArmPose home = new ArmPose(45.0, -85.0, 170.0, 90.0, 0.0, 0.0, 0.0);
    public static ArmPose carry = new ArmPose(0.0, -45.0, 135.0, 0.0, 0.0, 0.0, 0.0);

    /**
     * This method will construct an ArmPose where the gripper is centered on the target.
     * The target position is specified in cylindrical, robot-centric coordinates (r, alpha, z)
     * @param target
     * @return
     */
    public static ArmPose onTarget(CylindricalVector3D target){ // target is (r, alpha, z)
        double th0, th1, th2, th3, th4, th5, th6;
        th0 = target.theta; // angle to the target
        th1 = target.theta; // from the law of cosines !!!!!!!!!!
        th2 = 0; // from the law of cosines
        th3 = 0; // straight wrist
        th4 = 90 - th1 - th2; // for a level gripper, in degrees
        th5 = 0; // open gripper
        th6 = 0; // future gripper
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }

    public static ArmPose rotateTo(CylindricalVector3D target) {
        // use Robot class to transform from robot cylindrical coordinates to robot arm angles
        Robot robot = new Robot();
      //  robot.update(new Vector2D(target.z, target.rho), new Vector2D(0.0, 0.0));

        double th0, th1, th2, th3, th4, th5, th6;
        th0 = target.theta; // angle to the target
        th1=0;
        th2=0;
//        th1 = robot.theta1; // from the law of cosines
//        th2 = robot.theta2; // from the law of cosines
        th3 = 0; // spin wrist
        th4 = 90 - th1 - th2; // for a level gripper, in degrees -- soon to change to wrist flex
        th5 = 0; // open gripper -- soon to change to side-to-side gripper (up down)
        th6 = 0; // future gripper
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }

    public static ArmPose gripAndMoveTo(CylindricalVector3D target) {
        // use Robot class to transform from robot cylindrical coordinates to robot arm angles
        //robot.update(new Vector2D(target.z, target.rho), new Vector2D(0.0, 0.0));

        double th0, th1, th2, th3, th4, th5, th6;
        th1=0;
        th2=0;
        th0 = target.theta; // angle to the target
     //   th1 = robot.theta1; // from the law of cosines
     //   th2 = robot.theta2; // from the law of cosines
        th3 = 0; // spin wrist
        th4 = 90 - th1 - th2; // for a level gripper, in degrees -- soon to change to wrist flex
        th5 = 0; // open gripper -- side-to-side gripper (up down)
        th6 = 180; // future gripper -- is this a grip???
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }

    public static ArmPose releaseGripAndMoveTo(CylindricalVector3D target) {
        // use Robot class to transform from robot cylindrical coordinates to robot arm angles
        Robot robot = new Robot();
      //  robot.update(new Vector2D(target.z, target.rho), new Vector2D(0.0, 0.0));

        double th0, th1, th2, th3, th4, th5, th6;
        th0 = target.theta; // angle to the target
        th1=0;
        th2=0;
   //     th1 = robot.theta1; // from the law of cosines
   //     th2 = robot.theta2; // from the law of cosines
        th3 = 0; // spin wrist
        th4 = 90 - th1 - th2; // for a level gripper, in degrees -- soon to change to wrist flex
        th5 = 0; // open gripper -- soon to change to side-to-side gripper (up down)
        th6 = 0; // future gripper -- 0 is release grip
        return new ArmPose(th0, th1, th2, th3, th4, th5, th6);
    }
}
