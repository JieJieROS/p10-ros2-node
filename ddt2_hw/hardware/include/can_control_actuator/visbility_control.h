#ifndef DDT2_HW__VISIBILITY_CONTROL_H_
#define DDT2_HW__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DDT2_HW_EXPORT __attribute__((dllexport))
#define DDT2_HW_IMPORT __attribute__((dllimport))
#else
#define DDT2_HW_EXPORT __declspec(dllexport)
#define DDT2_HW_IMPORT __declspec(dllimport)
#endif
#ifdef DDT2_HW_BUILDING_DLL
#define DDT2_HW_PUBLIC DDT2_HW_EXPORT
#else
#define DDT2_HW_PUBLIC DDT2_HW_IMPORT
#endif
#define DDT2_HW_PUBLIC_TYPE DDT2_HW_PUBLIC
#define DDT2_HW_LOCAL
#else
#define DDT2_HW_EXPORT __attribute__((visibility("default")))
#define DDT2_HW_IMPORT
#if __GNUC__ >= 4
#define DDT2_HW_PUBLIC __attribute__((visibility("default")))
#define DDT2_HW_LOCAL __attribute__((visibility("hidden")))
#else
#define DDT2_HW_PUBLIC
#define DDT2_HW_LOCAL
#endif
#define DDT2_HW_PUBLIC_TYPE
#endif

#endif  // DDT2_HW__VISIBILITY_CONTROL_H_
