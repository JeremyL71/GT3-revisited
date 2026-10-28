#include "platform/mac_dialog.h"

#include <SDL3/SDL.h>

#if defined(SDL_PLATFORM_MACOS)
#include <objc/message.h>
#include <objc/runtime.h>
#endif

namespace gt3r::platform {

#if defined(SDL_PLATFORM_MACOS)
namespace {

id send(id obj, const char* sel) { return reinterpret_cast<id (*)(id, SEL)>(objc_msgSend)(obj, sel_registerName(sel)); }
void call(id obj, const char* sel) { reinterpret_cast<void (*)(id, SEL)>(objc_msgSend)(obj, sel_registerName(sel)); }
void call(id obj, const char* sel, id arg) {
    reinterpret_cast<void (*)(id, SEL, id)>(objc_msgSend)(obj, sel_registerName(sel), arg);
}
void call(id obj, const char* sel, BOOL arg) {
    reinterpret_cast<void (*)(id, SEL, BOOL)>(objc_msgSend)(obj, sel_registerName(sel), arg);
}
bool ask(id obj, const char* sel) { return reinterpret_cast<BOOL (*)(id, SEL)>(objc_msgSend)(obj, sel_registerName(sel)); }
bool responds(id obj, const char* sel) {
    return reinterpret_cast<BOOL (*)(id, SEL, SEL)>(objc_msgSend)(obj, sel_registerName("respondsToSelector:"),
                                                                 sel_registerName(sel));
}
id cls(const char* name) { return reinterpret_cast<id>(objc_getClass(name)); }
id ns_string(const std::string& s) {
    return reinterpret_cast<id (*)(id, SEL, const char*)>(objc_msgSend)(cls("NSString"), sel_registerName("stringWithUTF8String:"),
                                                                       s.c_str());
}

id ns_window_of(SDL_Window* window) {
    return window ? static_cast<id>(SDL_GetPointerProperty(SDL_GetWindowProperties(window), SDL_PROP_WINDOW_COCOA_WINDOW_POINTER, nullptr))
                  : nullptr;
}

} // namespace
#endif

bool show_file_sheet(SDL_Window* window, bool save, const std::string& folder, const std::string& name,
                     std::function<void(std::string path)> done) {
#if defined(SDL_PLATFORM_MACOS)
    id ns_window = ns_window_of(window);
    if (!ns_window || send(ns_window, "attachedSheet")) return false;
    id panel = send(cls(save ? "NSSavePanel" : "NSOpenPanel"), save ? "savePanel" : "openPanel");
    if (!panel) return false;
    call(panel, "retain"); // until its handler ran
    if (!save) {
        call(panel, "setCanChooseFiles:", YES);
        call(panel, "setCanChooseDirectories:", NO);
        call(panel, "setAllowsMultipleSelection:", NO);
    }
    if (!folder.empty())
        call(panel, "setDirectoryURL:",
             reinterpret_cast<id (*)(id, SEL, id, BOOL)>(objc_msgSend)(cls("NSURL"), sel_registerName("fileURLWithPath:isDirectory:"),
                                                                     ns_string(folder), YES));
    if (save && !name.empty()) call(panel, "setNameFieldStringValue:", ns_string(name));
    void (^handler)(long) = ^(long response) {
        std::string path;
        if (response == 1) { // NSModalResponseOK
            id url = save ? send(panel, "URL") : send(send(panel, "URLs"), "firstObject");
            id p = url ? send(url, "path") : nullptr;
            const char* s = p ? reinterpret_cast<const char* (*)(id, SEL)>(objc_msgSend)(p, sel_registerName("UTF8String")) : nullptr;
            if (s) path = s;
        }
        done(path);
        call(panel, "release");
    };
    reinterpret_cast<void (*)(id, SEL, id, void (^)(long))>(objc_msgSend)(
        panel, sel_registerName("beginSheetModalForWindow:completionHandler:"), ns_window, handler);
    return true;
#else
    (void)window, (void)save, (void)folder, (void)name, (void)done;
    return false;
#endif
}

void refocus_after_dialog(SDL_Window* window) {
#if defined(SDL_PLATFORM_MACOS)
    id ns_window = ns_window_of(window);
    if (!ns_window || send(ns_window, "attachedSheet")) return; // (another sheet: it has the keys)
    id app = send(cls("NSApplication"), "sharedApplication");
    if (app && !ask(app, "isActive")) {
        if (responds(app, "activate")) call(app, "activate"); // macOS 14 on
        else call(app, "activateIgnoringOtherApps:", YES);
    }
    if (!ask(ns_window, "isKeyWindow")) call(ns_window, "makeKeyAndOrderFront:", static_cast<id>(nullptr));
#else
    (void)window;
#endif
}

bool cancel_window_sheet(SDL_Window* window) {
#if defined(SDL_PLATFORM_MACOS)
    id ns_window = ns_window_of(window);
    id sheet = ns_window ? send(ns_window, "attachedSheet") : nullptr;
    if (!sheet) return false;
    // A file panel's cancel: (its Cancel button); any other sheet ends with NSModalResponseCancel.
    if (responds(sheet, "cancel:"))
        call(sheet, "cancel:", static_cast<id>(nullptr));
    else
        reinterpret_cast<void (*)(id, SEL, id, long)>(objc_msgSend)(ns_window, sel_registerName("endSheet:returnCode:"), sheet, 0L);
    return true;
#else
    (void)window;
    return false;
#endif
}

} // namespace gt3r::platform
