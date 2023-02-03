package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.RsContext;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Vector3D;

public class CameraWrapper implements WebcamName {
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

    public double[] scanForPole() {
        if (cameraEnabled) {
            return scanForPoleJNI();
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

    private native double[] scanForConeJNI();

    private native double[] scanForPoleJNI();

    @NonNull
    @Override
    public SerialNumber getSerialNumber() {
        return null;
    }

    @Nullable
    @Override
    public String getUsbDeviceNameIfAttached() {
        return "D405";
    }

    @Override
    public boolean isAttached() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "D405";
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    @Override
    public boolean isWebcam() {
        return false;
    }

    @Override
    public boolean isCameraDirection() {
        return false;
    }

    @Override
    public boolean isSwitchable() {
        return false;
    }

    @Override
    public boolean isUnknown() {
        return false;
    }

    @Override
    public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation<? extends Consumer<Boolean>> continuation) {

    }

    @Override
    public boolean requestCameraPermission(Deadline deadline) {
        return false;
    }

    @Override
    public CameraCharacteristics getCameraCharacteristics() {
        return null;
    }

}
