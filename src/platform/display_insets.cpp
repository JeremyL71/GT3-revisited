#include "platform/display_insets.h"

#include <SDL3/SDL.h>

#if defined(SDL_PLATFORM_ANDROID)
#include <jni.h>
#endif

namespace gt3r::platform {

std::optional<DisplayInsets> query_display_insets() {
#if defined(SDL_PLATFORM_ANDROID)
    auto* env = static_cast<JNIEnv*>(SDL_GetAndroidJNIEnv());
    auto activity = static_cast<jobject>(SDL_GetAndroidActivity());
    if (!env || !activity) return std::nullopt;
    std::optional<DisplayInsets> out;
    jclass cls = env->GetObjectClass(activity);
    // GT3Activity.displayInsets(): int[5] {left, top, right, bottom, corner radius} or null.
    jmethodID m = cls ? env->GetMethodID(cls, "displayInsets", "()[I") : nullptr;
    if (m) {
        auto arr = static_cast<jintArray>(env->CallObjectMethod(activity, m));
        if (env->ExceptionCheck()) env->ExceptionClear();
        else if (arr && env->GetArrayLength(arr) >= 5) {
            jint v[5];
            env->GetIntArrayRegion(arr, 0, 5, v);
            out = DisplayInsets{v[0], v[1], v[2], v[3], v[4]};
        }
        if (arr) env->DeleteLocalRef(arr);
    } else if (env->ExceptionCheck()) {
        env->ExceptionClear();
    }
    if (cls) env->DeleteLocalRef(cls);
    env->DeleteLocalRef(activity);
    return out;
#else
    return std::nullopt;
#endif
}

} // namespace gt3r::platform
