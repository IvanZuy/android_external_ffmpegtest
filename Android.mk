LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE          := libffmpeg.mx
LOCAL_MODULE_SUFFIX   := .so
LOCAL_MODULE_CLASS    := SHARED_LIBRARIES
LOCAL_MODULE_TAGS     := eng
LOCAL_PRELINK_MODULE  := false
LOCAL_MODULE_PATH     := system/lib
LOCAL_SRC_FILES       := libs/1/libffmpeg.mx.so
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= ffmpegtest.cpp \
                  utils/Utils.cpp

LOCAL_MODULE:= ffmpegtest

LOCAL_CFLAGS :=

LOCAL_C_INCLUDES += external/ffmpeg \
                    external/ffmpeg/android/include \
                    device/fsl-proprietary/include/ \
                    $(LOCAL_PATH)/utils

LOCAL_SHARED_LIBRARIES := libcutils \
                          libutils \
                          libffmpeg.mx \
                          libmedia \
                          libbinder \
                          libgui \
                          libui \
                          libstagefright_foundation \
                          libstagefright \
                          libg2d

LOCAL_MODULE_TAGS := eng

include $(BUILD_EXECUTABLE)
