#include <jni.h>
#include <string>
#include <android/log.h>

#include "include/librealsense2/rs.hpp"

#define  LOG_TAG    "ftcrobotcontroller.cpp"
#define  ALOG(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  ELOG(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)


// Write C++ code here.
//
// Do not forget to dynamically load the C++ library into your application.
//
// For instance,
//
// In MainActivity.java:
//    static {
//       System.loadLibrary("ftcrobotcontroller");
//    }
//
// Or, in MainActivity.kt:
//    companion object {
//      init {
//         System.loadLibrary("ftcrobotcontroller")
//      }
//    }
extern "C"
JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_TestJNIWrapper_stringFromJNI(JNIEnv *env, jobject thiz) {
    {
        std::string hello = "Hello from C++";
        return env->NewStringUTF(hello.c_str());
    }
}
extern "C"
JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_cameraStringFromJNI(JNIEnv *env, jobject thiz) {
    std::string hello = "Hello from Camera C++";
    return env->NewStringUTF(hello.c_str());
}
extern "C"
JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_nGetLibrealsenseVersionFromJNI(JNIEnv *env,
                                                                                 jclass clazz) {
    return (*env).NewStringUTF(RS2_API_VERSION_STR);
}
extern "C"
JNIEXPORT jint JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_nGetCamerasCountFromJNI(JNIEnv *env,
                                                                          jclass clazz) {
    rs2::context ctx;
    int number = ctx.query_devices().size();
    rs2::pipeline pipe;
    pipe.start();

    // Block program until frames arrive
    rs2::frameset frames = pipe.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();

    // Get the depth frame's dimensions
    auto width = depth.get_width();
    auto height = depth.get_height();

    // Query the distance from the camera to the object in the center of the image
    float dist_to_center = depth.get_distance(width / 2, height / 2);
    ELOG("Depth %f.", dist_to_center);

    return number;
}
extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_testMulticamFromJNI(JNIEnv *env, jobject thiz) {
    rs2::context ctx;
    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    int i = 1;
    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        std::string str = serial + " ";
        ELOG("Multicam %d.", i++);
    }

}