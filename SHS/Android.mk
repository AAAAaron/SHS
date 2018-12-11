LOCAL_PATH := $(call my-dir)


######################## ipscore #########################

include $(CLEAR_VARS)

NDK_MODULE_PATH :=$(LOCAL_PATH)

LOCAL_MODULE := pdrStep
#LOCAL_CFLAGS += -std=c++11 -D__DEBUG__
# LOCAL_CFLAGS += -pie -fPIE
# LOCAL_LDFLAGS += -pie -fPIE

LOCAL_SRC_FILES := src/pdrStep.cpp src/pdrsim.cpp  src/ahrsangle.cpp src/ahrsrobust.cpp src/gyro.cpp src/btwfilter.cpp  src/step.cpp src/ahrsqsmfbfilter src/ahrsqsmffilter src/phonerotateangle
LOCAL_C_INCLUDES := $(NDKROOT)/sources/cxx-stl/stlport/stlport/

LOCAL_C_INCLUDES += $(LOCAL_PATH)/include/
LOCAL_C_INCLUDES += /usr/include/eigen3/
# OpenCV
# OPENCV_CAMERA_MODULES:=on
# OPENCV_INSTALL_MODULES:=on
# OPENCV_LIB_TYPE:=SHARED
#LOCAL_C_INCLUDES += /usr/local/include/
#LOCAL_C_INCLUDES += /home/aaron/boost/
LOCAL_C_INCLUDES += /usr/include/
#_STLP_USE_BOOST_SUPPORT :=on
# LOCAL_EXPORT_C_INCLUDES :=$(LOCAL_PATH)/include

# LOCAL_SHARED_LIBRARIES := PDRSIM-prebuilt
# LOCAL_SHARED_LIBRARIES += prebuild
# LOCAL_PRELINK_MODULE := false
# LOCAL_LDFLAGS :=$(LOCAL_PATH)/lib/
# @LOCAL_LDLIBS
#LOCAL_SRC_FILES += logger/Logger.cpp logger/StandardLogger.cpp logger/FileLogger.cpp logger/LoggerWrap.cpp

#LOCAL_STATIC_LIBRARIES := mesch
LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -ldl -llog

include $(BUILD_SHARED_LIBRARY)
# include $(LOCAL_PATH)/lib/Android.mk
# $(call PDRSIM-prebuilt ,$(LOCAL_PATH)/lib/)
