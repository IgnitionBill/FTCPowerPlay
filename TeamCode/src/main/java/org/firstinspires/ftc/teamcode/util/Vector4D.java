package org.firstinspires.ftc.teamcode.util;

public class Vector4D {
    public double x;
    public double y;
    public double z;
    public double t;

    public void set(double x, double y, double z, double t){
        this.x = x;
        this.y = y;
        this.z = z;
        this.t = t;
    }

    public void set(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(Vector4D v){
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
        this.t = v.t;
    }

    public void set(Vector3D v){
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }
}
