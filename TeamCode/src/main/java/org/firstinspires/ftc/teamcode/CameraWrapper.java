package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;
import android.widget.TextView;

import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.RsContext;

import org.firstinspires.ftc.teamcode.util.Vector3D;

public class CameraWrapper {
    private boolean cameraEnabled = true;

    private RsContext mRsContext;

    static {
        System.loadLibrary("ftcrobotcontroller");
    }

    public CameraWrapper(Context applicationContext) {

        Log.e("CameraWrapper", cameraStringFromJNI());

        //RsContext.init must be called once in the application's lifetime before any interaction with physical RealSense devices.
        //For multi activities applications use the application context instead of the activity context
        RsContext.init(applicationContext);
        Log.e("CameraWrapper", "Initialized RxContext");

        //Register to notifications regarding RealSense devices attach/detach events via the DeviceListener.
        mRsContext = new RsContext();
        mRsContext.setDevicesChangedCallback(new DeviceListener() {
            @Override
            public void onDeviceAttach() {
                Log.e("CameraWrapper", "Device is attached"); // this message does not display
                printMessage();
            }

            @Override
            public void onDeviceDetach() {
                cameraEnabled = false;
                Log.e("CameraWrapper", "Device is detached");
                printMessage();
            }
        });
        Log.e("CameraWrapper", "Camera initialized.");
        //recordWithCameraViaJNI(1);
        //testMulticamFromJNI();
        printMessage();
    }

    private void printMessage() {
        try {
            // Example of a call to native methods
            int cameraCount = nGetCamerasCountFromJNI();
            final String version = nGetLibrealsenseVersionFromJNI();
            final String cameraCountString;
            if (cameraCount == 0)
                cameraCountString = "No cameras are currently connected.";
            else
                cameraCountString = "Number of cameras connected: " + cameraCount;

            Log.e("CameraWrapper", "This app use librealsense: " + version + "\n" + cameraCountString);
//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                TextView tv = (TextView) findViewById(R.id.sample_text);
//                tv.setText("This app use librealsense: " + version + "\n" + cameraCountString);
//            }
//        });
        }
        catch (Exception e){
            Log.e("CameraWrapper", e.toString());
        }

    }

    public double[] scanForCone() {
        if (cameraEnabled) {
            return scanForConeJNI();
        }
        else {
            double[] failure = {0.0, 0.0, 0.0};
            return failure;
        }
    }



    /**
     * A native method that is implemented by the 'ftcrobotcontroller' native library,
     * which is packaged with this application.
     */
    private native String cameraStringFromJNI();

    private static native String nGetLibrealsenseVersionFromJNI();

    private static native int nGetCamerasCountFromJNI();

    private native void testMulticamFromJNI();

    private native void recordWithCameraViaJNI(int sec);

//    public static native Vector3D vectorToCone();

    private native double[] scanForConeJNI();
}
