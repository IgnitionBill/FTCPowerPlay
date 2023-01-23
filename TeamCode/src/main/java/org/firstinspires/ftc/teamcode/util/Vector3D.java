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

    public Vector3D add(double x, double y, double z){
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }

    public Vector3D add(Vector3D v){
        return add(v.x, v.y, v.z);
    }
}
