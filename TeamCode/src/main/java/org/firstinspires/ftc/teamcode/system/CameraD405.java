package org.firstinspires.ftc.teamcode.system;

import android.content.Context;

import android.os.Looper;
import android.widget.TextView;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.tfod.CameraInformation;
import org.firstinspires.ftc.robotcore.external.tfod.FrameConsumer;
import org.firstinspires.ftc.robotcore.external.tfod.FrameGenerator;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.api.UvcApiCameraCharacteristics;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import android.os.Handler;
import android.util.Log;

import com.intel.realsense.librealsense.Align;
import com.intel.realsense.librealsense.CameraInfo;
import com.intel.realsense.librealsense.Colorizer;
import com.intel.realsense.librealsense.Config;
import com.intel.realsense.librealsense.DecimationFilter;
import com.intel.realsense.librealsense.DeviceList;
import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.Frame;
import com.intel.realsense.librealsense.FrameReleaser;
import com.intel.realsense.librealsense.FrameSet;
import com.intel.realsense.librealsense.GLRsSurfaceView;
import com.intel.realsense.librealsense.HoleFillingFilter;
import com.intel.realsense.librealsense.Option;
import com.intel.realsense.librealsense.Pipeline;
import com.intel.realsense.librealsense.PipelineProfile;
import com.intel.realsense.librealsense.Pointcloud;
import com.intel.realsense.librealsense.RsContext;
import com.intel.realsense.librealsense.SpatialFilter;
import com.intel.realsense.librealsense.StreamFormat;
import com.intel.realsense.librealsense.StreamType;
import com.intel.realsense.librealsense.TemporalFilter;
import com.intel.realsense.librealsense.ThresholdFilter;

public class CameraD405 implements CameraName, FrameGenerator, Runnable {

    private static final String TAG = "CameraD405";
    private static final int PERMISSIONS_REQUEST_CAMERA = 0;

    private boolean mPermissionsGranted = false;

    private Context mAppContext;
    private TextView mBackGroundText;
    private GLRsSurfaceView mGLSurfaceViewOrg;
    private GLRsSurfaceView mGLSurfaceViewProcessed;
    private boolean mIsStreaming = false;
    private final Handler mHandler = new Handler(Looper.getMainLooper());

    private Pipeline mPipeline;

    //filters
    private Align mAlign;
    private Colorizer mColorizerOrg;
    private Colorizer mColorizerProcessed;
    private DecimationFilter mDecimationFilter;
    private HoleFillingFilter mHoleFillingFilter;
    private Pointcloud mPointcloud;
    private TemporalFilter mTemporalFilter;
    private ThresholdFilter mThresholdFilter;
    private SpatialFilter mSpatialFilter;

    private RsContext mRsContext;

    //Information for tfod
    FrameConsumer frameConsumer;
    CameraCharacteristics cameraCharacteristics;

    public CameraD405(Context appContext) {
        mAppContext = appContext;
        init();
    }

    private void init() {
        Log.d(TAG,"Initializing...");

        //RsContext.init must be called once in the application lifetime before any interaction with physical RealSense devices.
        //For multi activities applications use the application context instead of the activity context
        RsContext.init(mAppContext);

        //Register to notifications regarding RealSense devices attach/detach events via the DeviceListener.
        mRsContext = new RsContext();
        mRsContext.setDevicesChangedCallback(mListener);

        mPipeline = new Pipeline();

        //init filters
        mAlign = new Align(StreamType.COLOR);
        mColorizerOrg = new Colorizer();
        mColorizerProcessed = new Colorizer();
        mDecimationFilter = new DecimationFilter();
        mHoleFillingFilter = new HoleFillingFilter();
        mPointcloud = new Pointcloud();
        mTemporalFilter = new TemporalFilter();
        mThresholdFilter = new ThresholdFilter();
        mSpatialFilter = new SpatialFilter();

        //config filters
        mThresholdFilter.setValue(Option.MIN_DISTANCE, 0.1f);
        mThresholdFilter.setValue(Option.MAX_DISTANCE, 0.8f);

        mDecimationFilter.setValue(Option.FILTER_MAGNITUDE, 8);

        //tfod
        setCameraCharacteristics();

        try (DeviceList dl = mRsContext.queryDevices()) {
            if (dl.getDeviceCount() > 0) {
                showConnectLabel(false);
                start();
            }
        }
        Log.d(TAG,"Initialization completed.");
    }

    private void setCameraCharacteristics() {
        cameraCharacteristics = new UvcApiCameraCharacteristics();
    }

    private void showConnectLabel(final boolean state) {

//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                mBackGroundText.setVisibility(state ? View.VISIBLE : View.GONE);
//            }
//        });
    }

    private DeviceListener mListener = new DeviceListener() {
        @Override
        public void onDeviceAttach() {
            Log.e(TAG, "Device is attached"); // this message does not display
            showConnectLabel(false);
        }

        @Override
        public void onDeviceDetach() {
            Log.e(TAG, "Device is detached");
            showConnectLabel(true);
            stop();
        }
    };

//    Runnable mStreaming = new Runnable() {
//        @Override
//        public void run() {
//            try {
//                try(FrameReleaser fr = new FrameReleaser()){
//                    FrameSet frames = mPipeline.waitForFrames().releaseWith(fr);
//                    FrameSet orgSet = frames.applyFilter(mColorizerOrg).releaseWith(fr);
//                    FrameSet processedSet = frames.applyFilter(mDecimationFilter).releaseWith(fr).
//                            applyFilter(mHoleFillingFilter).releaseWith(fr).
//                            applyFilter(mTemporalFilter).releaseWith(fr).
//                            applyFilter(mSpatialFilter).releaseWith(fr).
//                            applyFilter(mThresholdFilter).releaseWith(fr).
//                            applyFilter(mColorizerProcessed).releaseWith(fr).
//                            applyFilter(mAlign).releaseWith(fr);
//                    try(Frame org = orgSet.first(StreamType.DEPTH, StreamFormat.RGB8).releaseWith(fr)){
//                        try(Frame processed = processedSet.first(StreamType.DEPTH, StreamFormat.RGB8).releaseWith(fr)){
//                            mGLSurfaceViewOrg.upload(org);
//                            mGLSurfaceViewProcessed.upload(processed);
//                        }
//                    }
//                }
//                mHandler.post(mStreaming);
//            }
//            catch (Exception e) {
//                Log.e(TAG, "streaming, error: " + e.getMessage());
//            }
//        }
//    };

    private void configAndStart() throws Exception {
        try (Config config = new Config()) {
            config.enableStream(StreamType.DEPTH, 640, 480);
            config.enableStream(StreamType.COLOR, 640, 480);
            // try statement needed here to release resources allocated by the Pipeline:start() method
            try (PipelineProfile pp = mPipeline.start(config)) {
            }
        }
    }

    private synchronized void start() {
        if (mIsStreaming)
            return;
        try {
            Log.d(TAG, "try start streaming");
            mGLSurfaceViewOrg.clear();
            mGLSurfaceViewProcessed.clear();
            configAndStart();
            mIsStreaming = true;
            mHandler.post(this);
            Log.d(TAG, "streaming started successfully");
        } catch (Exception e) {
            Log.d(TAG, "failed to start streaming");
        }
    }

    private synchronized void stop() {
        if (!mIsStreaming)
            return;
        try {
            Log.d(TAG, "try stop streaming");
            mIsStreaming = false;
            mHandler.removeCallbacks(this);
            mPipeline.stop();
            Log.d(TAG, "streaming stopped successfully");
            mGLSurfaceViewOrg.clear();
            mGLSurfaceViewProcessed.clear();
        } catch (Exception e) {
            Log.d(TAG, "failed to stop streaming");
            mPipeline = null;
            mColorizerOrg.close();
            mColorizerProcessed.close();
        }
    }

    @Override
    public CameraInformation getCameraInformation() {
        return new CameraInformation(640, 480, 0, 100f, 100f);
    }

    @Override
    public void setFrameConsumer(FrameConsumer frameConsumer) {
        this.frameConsumer = frameConsumer;
    }

    @Override
    public void run() {
        try {
            try (FrameReleaser fr = new FrameReleaser()) {
                FrameSet frames = mPipeline.waitForFrames().releaseWith(fr);
                FrameSet orgSet = frames.applyFilter(mColorizerOrg).releaseWith(fr);
                FrameSet processedSet = frames.applyFilter(mDecimationFilter).releaseWith(fr).
                        applyFilter(mHoleFillingFilter).releaseWith(fr).
                        applyFilter(mTemporalFilter).releaseWith(fr).
                        applyFilter(mSpatialFilter).releaseWith(fr).
                        applyFilter(mThresholdFilter).releaseWith(fr).
                        applyFilter(mColorizerProcessed).releaseWith(fr).
                        applyFilter(mAlign).releaseWith(fr);
                try (Frame org = orgSet.first(StreamType.DEPTH, StreamFormat.RGB8).releaseWith(fr)) {
                    try (Frame processed = processedSet.first(StreamType.DEPTH, StreamFormat.RGB8).releaseWith(fr)) {
                        mGLSurfaceViewOrg.upload(org);
                        mGLSurfaceViewProcessed.upload(processed);
                    }
                }
            }
            mHandler.post(this);
        } catch (Exception e) {
            Log.e(TAG, "streaming, error: " + e.getMessage());
        }
    }

    @Override
    public boolean isWebcam() {
        return true;
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
        return cameraCharacteristics;
    }
}
