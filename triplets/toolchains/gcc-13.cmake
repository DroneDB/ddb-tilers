# Chainloaded by x64-linux-ddbt-gcc13: the dependencies must be built by the same GCC (and so
# the same libstdc++) as our code, otherwise the link with g++-13 fails (__cxa_call_terminate).
set(CMAKE_C_COMPILER gcc-13)
set(CMAKE_CXX_COMPILER g++-13)
# The rest is vcpkg's own Linux toolchain, which a chainloaded file replaces: the target processor
# (libjpeg-turbo builds its SIMD code for it, and refuses an unknown one) and the triplet's flags
# (-fPIC). vcpkg passes its root to the builds of its ports.
if(DEFINED _VCPKG_ROOT_DIR)
    include("${_VCPKG_ROOT_DIR}/scripts/toolchains/linux.cmake")
endif()
