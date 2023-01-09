#include <jni.h>
#include <string>

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