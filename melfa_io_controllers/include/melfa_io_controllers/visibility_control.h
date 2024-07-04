#ifndef MELFA_FR_IO_CONTROLLERS__VISIBILITY_CONTROL_H_
#define MELFA_FR_IO_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MELFA_FR_IO_CONTROLLERS_EXPORT __attribute__((dllexport))
#define MELFA_FR_IO_CONTROLLERS_IMPORT __attribute__((dllimport))
#else
#define MELFA_FR_IO_CONTROLLERS_EXPORT __declspec(dllexport)
#define MELFA_FR_IO_CONTROLLERS_IMPORT __declspec(dllimport)
#endif
#ifdef MELFA_FR_IO_CONTROLLERS_BUILDING_DLL
#define MELFA_FR_IO_CONTROLLERS_PUBLIC MELFA_FR_IO_CONTROLLERS_EXPORT
#else
#define MELFA_FR_IO_CONTROLLERS_PUBLIC MELFA_FR_IO_CONTROLLERS_IMPORT
#endif
#define MELFA_FR_IO_CONTROLLERS_PUBLIC_TYPE MELFA_FR_IO_CONTROLLERS_PUBLIC
#define MELFA_FR_IO_CONTROLLERS_LOCAL
#else
#define MELFA_FR_IO_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define MELFA_FR_IO_CONTROLLERS_IMPORT
#if __GNUC__ >= 4
#define MELFA_FR_IO_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define MELFA_FR_IO_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define MELFA_FR_IO_CONTROLLERS_PUBLIC
#define MELFA_FR_IO_CONTROLLERS_LOCAL
#endif
#define MELFA_FR_IO_CONTROLLERS_PUBLIC_TYPE
#endif

#endif  // MELFA_FR_IO_CONTROLLERS__VISIBILITY_CONTROL_H_

