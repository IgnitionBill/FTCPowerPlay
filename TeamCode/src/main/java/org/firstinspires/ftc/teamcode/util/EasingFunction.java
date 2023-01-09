package org.firstinspires.ftc.teamcode.util;

// ratio is the ratio amount traveled from start to end (percentage done, except its a ratio)
// VERIFIED
public class EasingFunction {
    public static double linear(double startValue, double endValue, double ratio){
        if(ratio < 0.0)
            ratio = 0.0;
        if(ratio > 1.0)
            ratio = 1.0;
        return startValue + (endValue - startValue) * ratio;
    }
}
