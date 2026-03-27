#pragma once

#if defined(_MSC_VER)
# ifdef BUILDING_LIBRMCS
#  define LIBRMCS_API __declspec(dllexport)
# elif defined(STATIC_LINKING_LIBRMCS)
#  define LIBRMCS_API
# else
#  define LIBRMCS_API __declspec(dllimport)
# endif
#elif defined(__GNUC__) || defined(__clang__)
# ifdef BUILDING_LIBRMCS
#  define LIBRMCS_API __attribute__((visibility("default")))
# else
#  define LIBRMCS_API
# endif
#else
# define LIBRMCS_API
#endif
