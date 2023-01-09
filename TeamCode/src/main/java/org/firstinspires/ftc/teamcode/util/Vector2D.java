package org.firstinspires.ftc.teamcode.util;

public class Vector2D {
    private double x;
    private double y;

    public Vector2D(){

    }

    public Vector2D(double x, double y){
        set(x, y);
    }

    public double getX(){ return x; }

    public double getY(){ return y; }

    public Vector2D set(double x, double y){
        this.x = x;
        this.y = y;
        return this;
    }

    public Vector2D set(Vector2D vector2D){
        this.x = vector2D.x;
        this.y = vector2D.y;
        return this;
    }

    public Vector2D subtract(Vector2D vector2D){
        this.x -= vector2D.x;
        this.y -= vector2D.y;
        return this;
    }

    public Vector2D subtract(double dx, double dy){
        this.x -= dx;
        this.y -= dy;
        return this;
    }

    public Vector2D divideBy(double a){
        this.x = this.x/a;
        this.y = this.y/a;
        return this;
    }

    public Vector2D multiplyBy(double a){
        this.x = this.x*a;
        this.y = this.y*a;
        return this;
    }

    public Vector2D add(Vector2D vector2D){
        this.x += vector2D.x;
        this.y += vector2D.y;
        return this;
    }

    public Vector2D add(double dx, double dy){
        this.x += dx;
        this.y += dy;
        return this;
    }

    public Vector2D copy(){
        return new Vector2D(x, y);
    }

    public Vector2D rotate(double angle){
        double x = this.x;
        double y = this.y;
        this.x = x * Math.cos(Math.toRadians(angle)) - y * Math.sin(Math.toRadians(angle));
        this.y = x * Math.sin(Math.toRadians(angle)) + y * Math.cos(Math.toRadians(angle));
        return this;
    }

    public double magnitude(){
        return Math.sqrt(x*x + y*y);
    }

    public double bearingAngle(){
        return zeroTo360(Math.toDegrees(Math.atan2(y, x)));
    }

    public static double zeroTo360(double angle){
        while(angle > 360){
            angle -= 360;
        }
        while(angle < 0){
            angle += 360;
        }
        return angle;
    }

    public static double range180ToNeg180(double angle){
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180){
            angle += 360;
        }
        return angle;
    }

    public double distance(Vector2D p){
        return distance(this, p);
    }

    public static double distance(Vector2D p1, Vector2D p2){
        double dx = p2.x - p1.x;
        double dy = p2.y = p1.y;
        return Math.sqrt(dx*dx + dy*dy);
    }

    public String toString(){
        return String.format("[(%.2f), (%.2f)]", x, y);
    }
}
