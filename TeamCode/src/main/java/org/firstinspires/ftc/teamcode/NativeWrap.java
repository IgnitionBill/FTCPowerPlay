package org.firstinspires.ftc.teamcode;

public class NativeWrap {
    static {
//        System.loadLibrary("native");  DOESN'T WORK!!! AUTO INSERTED BY ANDROID STUDIO
        System.loadLibrary("ftcrobotcontroller"); // DOES WORK, INSERTED BY ME!
    }

    // Declare a native method sayHello() that receives no arguments and returns void
    public native void sayHello();
}
