package org.firstinspires.ftc.teamcode.util;

public class Vector3D {
    public double x;
    public double y;
    public double z;

    public Vector3D(){}

    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(Vector3D v){
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }

    public static Vector3D add(Vector3D v1, Vector3D v2){
        return new Vector3D(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
    }

    public Vector3D add(Vector3D v){
        return new Vector3D(v.x + x, v.y + y, v.z + z);
    }

    public Vector3D subtract(Vector3D v){
        return new Vector3D(-v.x + x, -v.y + y, -v.z + z);
    }

    public String toString(){
        return "(" + x + ", " + y + ", " + z + ")";
    }
}
