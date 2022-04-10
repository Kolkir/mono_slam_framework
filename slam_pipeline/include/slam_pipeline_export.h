
#ifndef SLAM_PIPELINE_EXPORT_H
#define SLAM_PIPELINE_EXPORT_H

#ifdef SLAM_PIPELINE_STATIC_DEFINE
#  define SLAM_PIPELINE_EXPORT
#  define SLAM_PIPELINE_NO_EXPORT
#else
#  ifndef SLAM_PIPELINE_EXPORT
#    ifdef slam_pipeline_EXPORTS
        /* We are building this library */
#      define SLAM_PIPELINE_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define SLAM_PIPELINE_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef SLAM_PIPELINE_NO_EXPORT
#    define SLAM_PIPELINE_NO_EXPORT 
#  endif
#endif

#ifndef SLAM_PIPELINE_DEPRECATED
#  define SLAM_PIPELINE_DEPRECATED __declspec(deprecated)
#endif

#ifndef SLAM_PIPELINE_DEPRECATED_EXPORT
#  define SLAM_PIPELINE_DEPRECATED_EXPORT SLAM_PIPELINE_EXPORT SLAM_PIPELINE_DEPRECATED
#endif

#ifndef SLAM_PIPELINE_DEPRECATED_NO_EXPORT
#  define SLAM_PIPELINE_DEPRECATED_NO_EXPORT SLAM_PIPELINE_NO_EXPORT SLAM_PIPELINE_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef SLAM_PIPELINE_NO_DEPRECATED
#    define SLAM_PIPELINE_NO_DEPRECATED
#  endif
#endif

#endif /* SLAM_PIPELINE_EXPORT_H */
