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

    public static CylindricalVector3D toCylindrical(Vector3D v){
        return new CylindricalVector3D(Math.sqrt(v.x*v.x + v.y*v.y), Math.toDegrees(Math.atan2(v.y, v.x)), v.z);
    }

    /**
     * Simple addition of cylindrical vectors
     */
    public static CylindricalVector3D add(CylindricalVector3D v1, CylindricalVector3D v2){
        double diff = Math.toRadians(v1.theta - v2.theta);
        double cosDiff = Math.cos(diff);
        double sinDiff = Math.sin(diff);
        return new CylindricalVector3D(
                Math.sqrt(v1.rho * v1.rho + v2.rho * v2.rho + 2 * v1.rho * v2.rho * cosDiff),
                v2.theta + Math.toDegrees(Math.atan2(v1.rho * sinDiff, v2.rho + v1.rho * cosDiff)),
                v1.z + v2.z
        );
    }

    public static void runTest(){
        // TODO: try numbers from -400 degrees to 400 degrees
        CylindricalVector3D v1 = new CylindricalVector3D(10, 45, 10);
        CylindricalVector3D v2 = new CylindricalVector3D(50, 10, 20);
        CylindricalVector3D sum = add(v1, v2);
        CylindricalVector3D sum2 = add(v2, v1); // should equal sum
        Vector3D cart1 = v1.toCartesian();
        Vector3D cart2 = v2.toCartesian();
        CylindricalVector3D v1fromCart = toCylindrical(cart1); // should equal v1
        CylindricalVector3D v2fromCart = toCylindrical(cart2); // should equal v2
        Vector3D cartSum = Vector3D.add(cart1, cart2);
        CylindricalVector3D sumFromCart = toCylindrical(cartSum); // should equal sum and sum2
    }
}
