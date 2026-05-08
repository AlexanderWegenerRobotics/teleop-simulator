#pragma once
// Force-included on Windows via CMake /FI flag.
// Guarantees winsock2.h is the very first Windows header in every TU,
// preventing the classic winsock.h vs winsock2.h redefinition conflict.
#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #include <timeapi.h>
#endif
