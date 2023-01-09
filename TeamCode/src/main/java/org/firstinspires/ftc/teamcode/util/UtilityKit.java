package org.firstinspires.ftc.teamcode.util;

public class UtilityKit {
    static double wheelCirCm = Math.PI*9.6; // cm
    static double wheelCirIn = Math.PI*3.779; // in

    //        GoBilda 5203-2402-0003 DC motor 3.7:1 ratio, 1620 rpm
//        GoBilda 3204-0001-0002 worm gear ratio: 28:1
//        PPR = 103.6 pulse/revolution at the motor
//        103.6/360 = 0.288 ticks/degree at the worm gear
//        .288 x 28 = 8.06 ticks/degree at the joint
    public final static double ppr = 103.6;
    public final static double wormGearRatio = 28;
    public final static double ticksPerDegreeAtWormGear = ppr / 360;
    public final static double ticksPerDegreeAtJoint = ticksPerDegreeAtWormGear * wormGearRatio;

    public static double limitToRange(double input, double min, double max){
        if(input > max) return max;
        if(input < min) return min;
        return input;
    }

    public static int limitToRange(int input, int min, int max){
        if(input > max) return max;
        if(input < min) return min;
        return input;
    }

    static public int driveDistanceToTicks (double distance, UnitOfDistance unitOfDistance) { // cm
        if (unitOfDistance == UnitOfDistance.CM) {
            return driveDegreesToTicks(distance * (360 / wheelCirCm));
        }
        else if (unitOfDistance == UnitOfDistance.IN) {
            return driveDegreesToTicks(distance * (360 / wheelCirIn));
        }
        else {
            return 0;
        }
    }

    static public double driveTicksToDegrees(int ticks) {
        return 360/537.7*ticks;
    }

    static public int driveDegreesToTicks(double degrees) {
        return (int)(537.7/360*degrees);
    }

    static public int armDegreesToTicks(double degrees) { return (int)(ticksPerDegreeAtJoint*degrees);}

    static public double armTicksToDegrees(double ticks) {return ticksPerDegreeAtJoint/ticks;} //TODO: Verify this line

    static public int grabberDegreesToTicks(double degrees) { return (int)((5281.1)/360*degrees);}

    static public double sin(double n, UnitOfAngle unitOfAngle) {
        if (unitOfAngle == UnitOfAngle.DEGREES) {
            return Math.sin(Math.toRadians(n));
        }
        else {
            return Math.sin(n);
        }
    }

    static public double cos(double n, UnitOfAngle unitOfAngle) {
        if (unitOfAngle == UnitOfAngle.DEGREES) {
            return Math.cos(Math.toRadians(n));
        }
        else {
            return Math.cos(n);
        }
    }

    static public double tan(double n, UnitOfAngle unitOfAngle) {
        if (unitOfAngle == UnitOfAngle.DEGREES) {
            return Math.tan(Math.toRadians(n));
        }
        else {
            return Math.tan(n);
        }
    }

    static public double asin(double n, UnitOfAngle unitOfAngle) {
        if (unitOfAngle == UnitOfAngle.DEGREES) {
            return Math.toDegrees(Math.asin(n));
        }
        else {
            return Math.asin(n);
        }
    }

    static public double acos(double n, UnitOfAngle unitOfAngle) {
        if (unitOfAngle == UnitOfAngle.DEGREES) {
            return Math.toDegrees(Math.acos(n));
        }
        else {
            return Math.acos(n);
        }
    }

    static public double atan(double n, UnitOfAngle unitOfAngle) {
        if (unitOfAngle == UnitOfAngle.DEGREES) {
            return Math.toDegrees(Math.atan(n));
        }
        else {
            return Math.atan(n);
        }
    }

    static public double inToCm(double n) {return n*2.54;}

    static public double cmToIn(double n) {return n/2.54;}
}
