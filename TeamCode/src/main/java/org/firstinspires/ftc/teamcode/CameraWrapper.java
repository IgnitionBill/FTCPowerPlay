package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;
import android.widget.TextView;

import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.RsContext;

public class CameraWrapper {
    private RsContext mRsContext;

    static {
        System.loadLibrary("ftcrobotcontroller");
    }

    public CameraWrapper(Context applicationContext) {

        Log.e("TestCameraWrapper", cameraStringFromJNI());

        //RsContext.init must be called once in the application's lifetime before any interaction with physical RealSense devices.
        //For multi activities applications use the application context instead of the activity context
        RsContext.init(applicationContext);
        Log.e("TestCameraWrapper", "Initialized RxContext");

        //Register to notifications regarding RealSense devices attach/detach events via the DeviceListener.
        mRsContext = new RsContext();
        mRsContext.setDevicesChangedCallback(new DeviceListener() {
            @Override
            public void onDeviceAttach() {
                printMessage();
                Log.e("TestCameraWrapper", "Device is attached");
            }

            @Override
            public void onDeviceDetach() {
                printMessage();
                Log.e("TestCameraWrapper", "Device is detached");
            }
        });
        Log.e("TestCameraWrapper", "Outside of device listener callback.");
        printMessage();
    }

    private void printMessage() {
        // Example of a call to native methods
        int cameraCount = nGetCamerasCountFromJNI();
        final String version = nGetLibrealsenseVersionFromJNI();
        final String cameraCountString;
        if(cameraCount == 0)
            cameraCountString = "No cameras are currently connected.";
        else
            cameraCountString = "Camera is connected";

        Log.e("TestCameraWrapper", "This app use librealsense: " + version + "\n" + cameraCountString);
//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                TextView tv = (TextView) findViewById(R.id.sample_text);
//                tv.setText("This app use librealsense: " + version + "\n" + cameraCountString);
//            }
//        });
    }


    /**
     * A native method that is implemented by the 'ftcrobotcontroller' native library,
     * which is packaged with this application.
     */
    public native String cameraStringFromJNI();

    private static native String nGetLibrealsenseVersionFromJNI();

    private static native int nGetCamerasCountFromJNI();
}
