//
// Created by shengyp on 2018/10/24.
//

#ifndef _PDRSTEP_LIB_H
#define _PDRSTEP_LIB_H

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_initJniPdrStep(JNIEnv *, jclass);

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setJniPdrStepCallback(JNIEnv *, jclass, jobject object);

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setJniPressureCallback(JNIEnv *, jclass, jobject object);

JNIEXPORT jboolean JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_nextStepEx(JNIEnv *env, jclass, jdouble gyro_x,jdouble gyro_y,jdouble gyro_z,jdouble linear_acc_x,jdouble linear_acc_y,jdouble linear_acc_z,jdouble gx,jdouble gy,jdouble gz,jdouble mag_x,jdouble mag_y,jdouble mag_z,jdouble time_r);

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_releaseJniPdrLib(JNIEnv *, jclass);

JNIEXPORT jstring JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getVersionFromJNI(JNIEnv *, jclass);


JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getX(JNIEnv *, jclass);

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getY(JNIEnv *, jclass);

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getSL(JNIEnv *, jclass);

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getYAW(JNIEnv *, jclass);

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getDetaAngle(JNIEnv *, jclass);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setMagOffset(JNIEnv *, jclass ,jdouble offset);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setXY(JNIEnv *, jclass ,jdouble x,jdouble y);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setXYYaw(JNIEnv *, jclass ,jdouble x,jdouble y,jdouble yaw);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setYaw(JNIEnv *, jclass ,jdouble yaw);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitXYYaw(JNIEnv *, jclass ,jdouble x,jdouble y,jdouble yaw);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitXY(JNIEnv *, jclass ,jdouble x,jdouble y);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitXYYawFloor(JNIEnv *, jclass ,jdouble x,jdouble y,jdouble yaw,jint curfloorIndex);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitFloor(JNIEnv *, jclass ,jint curfloorIndex);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_chooseMethod(JNIEnv *, jclass ,jint index);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setStep(JNIEnv *, jclass,jdouble f_scale,jdouble s_scale);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setOutAngle(JNIEnv *, jclass,jint index);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setPdrEkf(JNIEnv *, jclass);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_floorModuleAddFloorHeight(JNIEnv *, jclass,jdouble floor_height);

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_floorModuleUpdateFloorHeightMatrix(JNIEnv *, jclass);

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_floorModuleAddData(JNIEnv *, jclass ,jdouble acc_norm,jdouble orientation,jdouble pressure,jdouble time_r);

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setCurrentFloorIndex(JNIEnv *, jclass,jint floorId);

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getCurrentFloorIndex(JNIEnv *, jclass);

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getFloorK(JNIEnv *, jclass);

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getFloorAccStd(JNIEnv *, jclass);

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getFloorType(JNIEnv *, jclass);
#ifdef __cplusplus
}
#endif
#endif //HOSPITALAPP_PDRSTEP_LIB_H
