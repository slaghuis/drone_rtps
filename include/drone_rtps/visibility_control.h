#ifndef DRONE__VISIBILITY_H_
#define DRONE__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define DRONE_EXPORT __attribute__ ((dllexport))
    #define DRONE_IMPORT __attribute__ ((dllimport))
  #else
    #define DRONE_EXPORT __declspec(dllexport)
    #define DRONE_IMPORT __declspec(dllimport)
  #endif

  #ifdef DRONE_DLL
    #define DRONE_PUBLIC DRONE_EXPORT
  #else
    #define DRONE_PUBLIC DRONE_IMPORT
  #endif

  #define DRONE_PUBLIC_TYPE DRONE_PUBLIC

  #define DRONE_LOCAL

#else

  #define DRONE_EXPORT __attribute__ ((visibility("default")))
  #define DRONE_IMPORT

  #if __GNUC__ >= 4
    #define DRONE_PUBLIC __attribute__ ((visibility("default")))
    #define DRONE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DRONE_PUBLIC
    #define DRONE_LOCAL
  #endif

  #define DRONE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DRONE__VISIBILITY_H_
