# Windows x64 cross build with MinGW-w64 (GCC), for checking Windows code paths and making
# Windows executables from macOS or Linux (docs/dev/PLATFORMS.md "Windows"):
#   cmake -S . -B build-mingw -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64.cmake -DCMAKE_BUILD_TYPE=Release \
#         -DGT3R_FETCH_SDL=ON -DCMAKE_EXE_LINKER_FLAGS="-static -static-libgcc -static-libstdc++"
# GT3R_FETCH_SDL=ON matters: without it CMake finds the host's own SDL3 (e.g. Homebrew's).
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER x86_64-w64-mingw32-windres)
# Homebrew (macOS) keeps the MinGW sysroot here; distributions use /usr/x86_64-w64-mingw32.
foreach(root /opt/homebrew/opt/mingw-w64/toolchain-x86_64 /usr/local/opt/mingw-w64/toolchain-x86_64 /usr/x86_64-w64-mingw32)
  if(EXISTS ${root})
    list(APPEND CMAKE_FIND_ROOT_PATH ${root})
  endif()
endforeach()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
