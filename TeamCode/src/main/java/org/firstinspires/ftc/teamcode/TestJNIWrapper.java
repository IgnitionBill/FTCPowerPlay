package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

public class TestJNIWrapper {
    static {
       System.loadLibrary("ftcrobotcontroller");
    }

public TestJNIWrapper(Context applicationContext){
        Log.e("TestJNI", stringFromJNI());
}

    /**
     * A native method that is implemented by the 'ftcrobotcontroller' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
}
