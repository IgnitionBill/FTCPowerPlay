package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

//import com.intel.realsense.librealsense.DeviceListener;
//import com.intel.realsense.librealsense.RsContext;

public class TestJNIWrapper {
    static {
       System.loadLibrary("ftcrobotcontroller");
    }

//    private RsContext mRsContext;

    // Used to load the 'cameras' library on application startup.
//    static {
//        System.loadLibrary("cameras");
//    }

public TestJNIWrapper(Context applicationContext){
        Log.e("TestJNI", stringFromJNI());

    //RsContext.init must be called once in the application's lifetime before any interaction with physical RealSense devices.
    //For multi activities applications use the application context instead of the activity context
//    RsContext.init(applicationContext);

//    printMessage();

    //Register to notifications regarding RealSense devices attach/detach events via the DeviceListener.
//    mRsContext = new RsContext();
//    mRsContext.setDevicesChangedCallback(new DeviceListener() {
//        @Override
//        public void onDeviceAttach() {
////            printMessage();
//        }
//
//        @Override
//        public void onDeviceDetach() {
////            printMessage();
//        }
//    });
}

//    private void printMessage(){
//        // Example of a call to native methods
//        int cameraCount = 0;//nGetCamerasCountFromJNI();
//        final String version = "zip"; //nGetLibrealsenseVersionFromJNI();
//        final String cameraCountString;
//        if(cameraCount == 0)
//            cameraCountString = "No cameras are currently connected.";
//        else
//            cameraCountString = "Camera is connected";
//
//        Log.e("Camera", cameraCountString);
//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                TextView tv = (TextView) findViewById(R.id.sample_text);
//                tv.setText("This app use librealsense: " + version + "\n" + cameraCountString);
//            }
//        });
//    }

    /**
     * A native method that is implemented by the 'ftcrobotcontroller' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
}
