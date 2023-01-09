package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

public class CameraWrapper {
    static {
       System.loadLibrary("ftcrobotcontroller");
    }

public CameraWrapper(Context applicationContext){
        Log.e("TestJNI", cameraStringFromJNI());
}

    /**
     * A native method that is implemented by the 'ftcrobotcontroller' native library,
     * which is packaged with this application.
     */
    public native String cameraStringFromJNI();
}
