#include <jni.h>
#include <string>
#include "include/librealsense2/rs.hpp"

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
    return ctx.query_devices().size();;
}