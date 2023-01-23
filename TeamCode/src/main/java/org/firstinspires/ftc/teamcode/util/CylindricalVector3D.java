package org.firstinspires.ftc.teamcode.util;

public class CylindricalVector3D {
    public double rho;
    public double theta;
    public double z;

    public CylindricalVector3D(){

    }

    public CylindricalVector3D(double rho, double theta, double z){
        this.rho = rho;
        this.theta = theta;
        this.z = z;
    }

    public void set(double rho, double theta, double z){
        this.rho = rho;
        this.theta = theta;
        this.z = z;
    }

    /**
     * This version of cartesian coordinates has z pointing upward from the ground, x forward and
     * y to the left of the robot
     * @return
     */
    public Vector3D toCartesian(){
        double thRadians = Math.toRadians(theta);
        return new Vector3D(rho * Math.cos(thRadians), rho * Math.sin(thRadians), z);
    }

    public CylindricalVector3D toCylindrical(Vector3D v){
        return new CylindricalVector3D(Math.sqrt(v.x*v.x + v.y*v.y), Math.toDegrees(Math.atan2(v.x, v.y)), v.z);
    }

    /**
     * Simple addition of cylindrical vectors, assuming they are pointed in the same theta
     */
    public CylindricalVector3D simpleAddition(CylindricalVector3D v){
        return new CylindricalVector3D(
                Math.sqrt(v.rho * v.rho + this.rho * this.rho + 2 * v.rho * this.rho * Math.cos(Math.toRadians(v.theta - this.theta))),
                theta,
                v.z + this.z
        );
    }
}
