#include <jni.h>
#include <string>
#include <android/log.h>
#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

#include "include/librealsense2/rs.hpp"

#define  LOG_TAG    "ftcrobotcontroller.cpp"
#define  ALOG(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  ELOG(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)


// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "include/stb_image_write.h"

// Helper function for writing metadata to disk as a csv file
//void metadata_to_csv(const rs2::frame& frm, const std::string& filename);


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
Java_org_firstinspires_ftc_teamcode_CameraWrapper_nGetLibrealsenseVersionFromJNI(JNIEnv *env, jclass clazz) {
    return (*env).NewStringUTF(RS2_API_VERSION_STR);
}
extern "C"
JNIEXPORT jint JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_nGetCamerasCountFromJNI(JNIEnv *env, jclass clazz) {
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
extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_recordWithCameraViaJNI(JNIEnv *env, jobject thiz,
                                                                         jint sec) {
    try {
        ELOG("Recording for %d", sec);
        // Declare depth colorizer for pretty visualization of depth data
        rs2::colorizer color_map;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Start streaming with default recommended configuration
        pipe.start();

        // Capture 30 frames to give autoexposure, etc. a chance to settle
        for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

        // Wait for the next set of frames from the camera. Now that autoexposure, etc.
        // has settled, we will write these to disk
        int j = 0;
        ELOG("Tossed 30 frames, entering loop %d", j);
        for (auto &&frame: pipe.wait_for_frames()) {
            j++;
            // We can only save video frames as pngs, so we skip the rest
            if (auto vf = frame.as<rs2::video_frame>()) {
                auto stream = frame.get_profile().stream_type();
                // Use the colorizer to get an rgb image for the depth stream
                if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

                // Write images to disk
                std::stringstream png_file;
                png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
                stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                               vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                std::cout << "Saved " << png_file.str() << std::endl;

                // Record per-frame metadata for UVC streams
//            std::stringstream csv_file;
//            csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
//                     << "-metadata.csv";
//            metadata_to_csv(vf, csv_file.str());
            }
            ELOG("Saving frame %d", j);
        }
        ELOG("Done recording %d", sec);
        return;
    }
    catch(const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return;
    }
    catch(const std::exception & e)
    {
        std::cerr << e.what() << std::endl;
        return;
    }
}

/////////////////////////////  SCAN FOR CONE //////////////////////////////////////
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_scanForConeJNI(JNIEnv *env, jobject thiz) {
    //ELOG("Scanning for cone CPP %d", __LINE__);
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

    ELOG("Scanning for cone CPP width %d", width);
    ELOG("Scanning for cone CPP height %d", height);
    // Depth output resolution: 1280/720
    // Depth accuracy: 2% at 50 cm
    // RGB resolution: 1280/720
    // RGB frame rate: 90 fps
    // FOV: H: 87, V: 58 // H: 1.5184, V: 1.0122 // (n * PI) / 180

//    float lCamera = 6.5; // cm // X distance to camera from base joint
//    float l0 = 12.5; // cm // X distance from center to base joint
//    float l1 = 33.5; // cm // Distance from base joint to lower joint
//    float l2 = 34; // cm // Distance from lower joint to wrist joint
//    float l3 = 18; // cm // Distance from wrist to center of gripper
//    float coneRadius = 2.5; // cm // Radius of cone
//
//    float maxRange = l1 + l2 + l3 - lCamera -
//                     coneRadius; // 76.5 cm // Maximum reach from camera to center of gripper

    int smallX = 0;
    int smallY = 0;
    float smallDepth = 0;

    // scan a rectangle around the center of the screen to find the closest point to the camera
    for (int j = height / 2; j < height; ++j) { // down to up
        for (int i = width * 3 / 8; i < width * 5 / 8; ++i) { // left to right
            float newDepth = depth.get_distance(i, j);
            if (smallDepth == 0) {
                smallDepth = newDepth;
                smallX = i;
                smallY = j;
            } else if (newDepth < smallDepth && newDepth != 0) {
                smallDepth = newDepth;
                smallX = i;
                smallY = j;
            }
        }
    }

    smallDepth = smallDepth * 100; // convert from meters to cm
    smallX = smallX - width / 2;
    smallY = height - smallY;  // reverse axis
    smallY = smallY - height / 2;

    float arcX = (87 * M_PI) / 180 * smallDepth; // cm // In parallel to the X axis
    float cmPerPixelX = arcX / width; // cm/pixel
    float arcY = (59 * M_PI) / 180 * smallDepth; // cm // In line to the Y axis
    float cmPerPixelY = arcY / height; // cm/pixel

    double x = cmPerPixelX * smallX;
    double y = cmPerPixelX * smallY;
    double z = smallDepth;
    double arr[3]={x,y,z};
    jdoubleArray ret = env->NewDoubleArray(3);
    env->SetDoubleArrayRegion(ret,0,3,arr);
    return ret;
}

//////////////////////////// SCAN FOR POLE ////////////////////////////////////
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_org_firstinspires_ftc_teamcode_CameraWrapper_scanForPoleJNI(JNIEnv *env, jobject thiz) {

    //ELOG("Scanning for cone CPP %d", __LINE__);
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

    ELOG("Scanning for cone CPP width %d", width);
    ELOG("Scanning for cone CPP height %d", height);
    // Depth output resolution: 1280/720
    // Depth accuracy: 2% at 50 cm
    // RGB resolution: 1280/720
    // RGB frame rate: 90 fps
    // FOV: H: 87, V: 58 // H: 1.5184, V: 1.0122 // (n * PI) / 180

    int smallX = 0; // position of the closest point
    int smallY = 0;
    float smallDepth = 0;
    int lastX = 0;
    int lastY = 0;
    float lastDepth = 30;

    // scan a rectangle around the center of the screen to find the closest point to the camera
    for (int j = height / 2; j < height; ++j) { // down to up
        for (int i = width * 3 / 8; i < width * 5 / 8; ++i) { // left to right
            float newDepth = depth.get_distance(i, j);
            if (smallDepth == 0) {
                smallDepth = newDepth;
                smallX = i;
                smallY = j;
            } else if (newDepth < smallDepth && newDepth != 0) {
                smallDepth = newDepth;
                smallX = i;
                smallY = j;
            }
        }
        // if the closest point on the line is much farther than the closest point on the last line,
        if(smallDepth > lastDepth + .005){
            // we have found the top and center
            break;
        }
        // store the closest point on the line
        lastX = smallX;
        lastY = smallY;
        lastDepth = smallDepth;
    }

    smallDepth = lastDepth * 100; // convert from meters to cm
    smallX = lastX - width / 2; // shift to the center of the screen
    smallY = height - smallY;  // reverse axis
    smallY = lastY - height / 2; // shift to the center of the screen

    float arcX = (87 * M_PI) / 180 * smallDepth; // cm // In parallel to the X axis
    float cmPerPixelX = arcX / width; // cm/pixel
    float arcY = (59 * M_PI) / 180 * smallDepth; // cm // In line to the Y axis
    float cmPerPixelY = arcY / height; // cm/pixel

    double x = cmPerPixelX * smallX;
    double y = cmPerPixelX * smallY;
    double z = smallDepth;
    double arr[3]={x,y,z};
    jdoubleArray ret = env->NewDoubleArray(3);
    env->SetDoubleArrayRegion(ret,0,3,arr);
    return ret;
}
