LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= ffmpegtest.c

LOCAL_MODULE:= ffmpegtest

LOCAL_CFLAGS :=

LOCAL_C_INCLUDES += external/ffmpeg \
                    external/ffmpeg/android/include

LOCAL_SHARED_LIBRARIES := libcutils \
                          libutils \
                          libavcodec \
                          libavutil

LOCAL_MODULE_TAGS := eng

include $(BUILD_EXECUTABLE)
