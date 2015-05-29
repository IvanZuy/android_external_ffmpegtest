LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= ffmpegtest.cpp

LOCAL_MODULE:= ffmpegtest

LOCAL_CFLAGS :=

LOCAL_C_INCLUDES += external/ffmpeg \
                    external/ffmpeg/android/include \
                    $(LOCAL_PATH) device/fsl-proprietary/include/

LOCAL_SHARED_LIBRARIES := libcutils \
                          libutils \
                          libavcodec \
                          libavutil \
                          libbinder \
                          libgui \
                          libui \
                          libg2d

LOCAL_MODULE_TAGS := eng

include $(BUILD_EXECUTABLE)
