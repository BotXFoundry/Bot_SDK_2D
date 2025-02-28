#ifndef BOT_SDK_CAN_VISIBILITY_CONTROL_HPP_
#define BOT_SDK_CAN_VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BOT_EXPORT __attribute__((dllexport))
#define BOT_IMPORT __attribute__((dllimport))
#else
#define BOT_EXPORT __declspec(dllexport)
#define BOT_IMPORT __declspec(dllimport)
#endif
#ifdef BOT_BUILDING_LIBRARY
#define BOT_PUBLIC BOT_EXPORT
#else
#define BOT_PUBLIC BOT_IMPORT
#endif
#define BOT_PUBLIC_TYPE BOT_PUBLIC
#define BOT_LOCAL
#else
#define BOT_EXPORT __attribute__((visibility("default")))
#define BOT_IMPORT
#if __GNUC__ >= 4
#define BOT_PUBLIC __attribute__((visibility("default")))
#define BOT_LOCAL __attribute__((visibility("hidden")))
#else
#define BOT_PUBLIC
#define BOT_LOCAL
#endif
#define BOT_PUBLIC_TYPE
#endif

#endif  // BOT_SDK_CAN_VISIBILITY_CONTROL_HPP_
