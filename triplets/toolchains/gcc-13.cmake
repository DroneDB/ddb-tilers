# Chainloaded by x64-linux-ddbt-gcc13: the dependencies must be built by the same GCC (and so
# the same libstdc++) as our code, otherwise the link with g++-13 fails (__cxa_call_terminate).
set(CMAKE_C_COMPILER gcc-13)
set(CMAKE_CXX_COMPILER g++-13)
