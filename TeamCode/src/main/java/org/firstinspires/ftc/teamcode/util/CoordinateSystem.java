package org.firstinspires.ftc.teamcode.util;

public class CoordinateSystem {
    Vector3D o; // the origin
    Matrix3x3 R; // a matrix with columns defined by a set of 3 orthogonal unit vectors
    Matrix3x3 RT; // the transpose of R, which is also the inverse for an orthonormal matrix

    // creates the standard coordinate system
    public CoordinateSystem(){
        o = new Vector3D();
        R = new Matrix3x3(new Vector3D(1, 0, 0),
                new Vector3D(0, 1, 0),
                new Vector3D(0, 0, 1));
        RT = R.copyTranspose();
    }

    // creates a coordinate system from an origin and set of orthogonal unit vectors
    public CoordinateSystem(Vector3D o, Vector3D u1, Vector3D u2, Vector3D u3){
        this.o = o;
        R = new Matrix3x3(u1, u2, u3);
        RT = R.copyTranspose();
    }

    // creates a coordinate system from an origin and matrix composed of orthogonal unit vectors
    public CoordinateSystem(Vector3D o, Matrix3x3 R){
        this.o = o;
        this.R = R;
        RT = R.copyTranspose();
    }

    public Vector3D transformTo(Vector3D v){
        return R.multiply(v).add(o);
    }

    public Vector3D transformFrom(Vector3D v){
        return RT.multiply(v.subtract(o));
    }
}
