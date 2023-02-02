package org.firstinspires.ftc.teamcode.util;

public class Matrix3x3 {
    double[][] m = new double[3][3];

    public Matrix3x3(){

    }

    /**
     * Creates a matrix by a set of column vectors
     * @param v1
     * @param v2
     * @param v3
     */
    public Matrix3x3(Vector3D v1, Vector3D v2, Vector3D v3){

        m[0][0] = v1.x;
        m[1][0] = v1.y;
        m[2][0] = v1.z;

        m[0][1] = v2.x;
        m[1][1] = v2.y;
        m[2][1] = v2.z;

        m[0][2] = v3.x;
        m[1][2] = v3.y;
        m[2][2] = v3.z;

    }

    public Matrix3x3 copy(){

        Matrix3x3 mat = new Matrix3x3();
        mat.m[0][0] = m[0][0];
        mat.m[1][0] = m[1][0];
        mat.m[2][0] = m[2][0];

        mat.m[0][1] = m[0][1];
        mat.m[1][1] = m[1][1];
        mat.m[2][1] = m[2][1];

        mat.m[0][2] = m[0][2];
        mat.m[1][2] = m[1][2];
        mat.m[2][2] = m[2][2];

        return mat;
    }


    /**
     * Creates a rotation matrix of amount angle in degrees about the normal vector
     * @param a
     * @param n
     * @return
     */
    public void setRotation(double a, Vector3D n){
        double cosA = Math.cos(Math.toRadians(a));
        double sinA = Math.sin(Math.toRadians(a));

        // first column
        m[0][0] = cosA + n.x * n.x * (1-cosA);
        m[1][0] = n.x * n.y * (1-cosA) + n.z * sinA;
        m[2][0] = n.x * n.z * (1-cosA) - n.y * sinA;

        // second column
        m[0][1] = n.x * n.y * (1-cosA) - n.z * sinA;
        m[1][1] = cosA + n.y * n.y * (1-cosA);
        m[2][1] = n.y * n.z * (1-cosA) + n.x * sinA;

        // third column
        m[0][2] = n.x * n.z * (1-cosA) + n.y * sinA;
        m[1][2] = n.y * n.z * (1-cosA) - n.x * sinA;
        m[2][2] = cosA + n.z * n.z * (1-cosA);

    }

    public Vector3D multiply(Vector3D v){
        // TODO: Print m to verify the matrix
        // TODO: Print v to verify the vector
        return new Vector3D(
                m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    public Matrix3x3 copyTranspose(){
        return copy().transpose();
    }

    public Matrix3x3 transpose(){
        double swap;
        swap = m[1][0];
        m[1][0] = m[0][1];
        m[0][1] = swap;

        swap = m[0][2];
        m[0][2] = m[2][0];
        m[2][0] = swap;

        swap = m[2][1];
        m[2][1] = m[1][2];
        m[1][2] = swap;
        return this;
    }

    /**
     * Multiplies two matrices into this matrix
     * @param m1
     * @param m2
     * @return
     */
    public Matrix3x3 multiply(Matrix3x3 m1, Matrix3x3 m2){
        return multiply(m1, m2, this);
    }

    public static Matrix3x3 multiply(Matrix3x3 m1, Matrix3x3 m2, Matrix3x3 r){
        r.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[0][1] * m2.m[1][0] + m1.m[0][2] * m2.m[2][0];
        r.m[0][1] = m1.m[0][0] * m2.m[0][1] + m1.m[0][1] * m2.m[1][1] + m1.m[0][2] * m2.m[2][1];
        r.m[0][2] = m1.m[0][0] * m2.m[0][2] + m1.m[0][1] * m2.m[1][2] + m1.m[0][2] * m2.m[2][2];

        r.m[1][0] = m1.m[1][0] * m2.m[0][0] + m1.m[1][1] * m2.m[1][0] + m1.m[1][2] * m2.m[2][0];
        r.m[1][1] = m1.m[1][0] * m2.m[0][1] + m1.m[1][1] * m2.m[1][1] + m1.m[1][2] * m2.m[2][1];
        r.m[1][2] = m1.m[1][0] * m2.m[0][2] + m1.m[1][1] * m2.m[1][2] + m1.m[1][2] * m2.m[2][2];

        r.m[2][0] = m1.m[2][0] * m2.m[0][0] + m1.m[2][1] * m2.m[1][0] + m1.m[2][2] * m2.m[2][0];
        r.m[2][1] = m1.m[2][0] * m2.m[0][1] + m1.m[2][1] * m2.m[1][1] + m1.m[2][2] * m2.m[2][1];
        r.m[2][2] = m1.m[2][0] * m2.m[0][2] + m1.m[2][1] * m2.m[1][2] + m1.m[2][2] * m2.m[2][2];

        return r;
    }

    public static void test(){
        Vector3D s1 = new Vector3D(1, 10, 100);
        Vector3D vx = new Vector3D(1, 0, 0); // x unit vector
        Vector3D vy = new Vector3D(0, 1, 0); // y unit vector
        Vector3D vz = new Vector3D(0, 0, 1); // z unit vector
        Matrix3x3 rotX = new Matrix3x3();
        rotX.setRotation(90, vx); // a 90 degree rotation about the x axis
        Vector3D r1 = rotX.multiply(s1); // should result in (1, -100, 10)
        Matrix3x3 invR1 = rotX.copyTranspose(); // should be the transpose
        Vector3D r2 = invR1.multiply(r1); // should result in s1
    }
}
