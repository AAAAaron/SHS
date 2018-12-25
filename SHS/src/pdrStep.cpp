//
// Created by shengyp on 2018/10/24.
//


#include "pdrStep.h"
#include "pdrsim.h"


extern "C"

#define LISTENER_PDR_FIELD_ID               ("pdrStepCallback")
#define LISTENER_PDR_JAVA_PATH              ("Lcom/xihe/newpdrsdk/locator/PdrStepCallbackInterface;")

#define CB_PDR_DATA_NAME                    ("OnPdrStepEvent")
#define CB_PDR_DATA_DESC                    ("(DDDDD)V")

#define LISTENER_PRESSURE_FIELD_ID          ("pressureCallback")
#define LISTENER_PRESSURE_JAVA_PATH         ("Lcom/xihe/newpdrsdk/locator/PressureCallbackInterface;")

#define CB_PRESSURE_DATA_NAME               ("OnFloorChangeEvent")
#define CB_PRESSURE_DATA_DESC               ("(IIDDI)V")


class JniPdrStepLib {

public:
    jobject lib_java;
    jobject listener_java;
    jmethodID OnEvent;
    JNIEnv *env;
};

JavaVM *vm = NULL;

JniPdrStepLib *pdr_lib = NULL;

JniPdrStepLib *floor_lib = NULL;

SHS::PDRSIM *step = NULL;

int setJNIPdrStepCallback(JNIEnv *env, jobject java_pdr_lib_obj) {

    jclass java_pdr_lib_cls;
    jfieldID listener_fid;
    jobject listener;

    pdr_lib->env = env;
    env->set
    pdr_lib->lib_java = env->NewGlobalRef(java_pdr_lib_obj);
    if (0 == pdr_lib->lib_java) {
        // LOGE("pdr_lib NewGlobalRef failed!");
        return -1;
    }

    java_pdr_lib_cls = env->GetObjectClass(java_pdr_lib_obj);
    listener_fid = env->GetFieldID(java_pdr_lib_cls, LISTENER_PDR_FIELD_ID, LISTENER_PDR_JAVA_PATH);
    if (0 == listener_fid) {
        // LOGE("GetFieldID listener_fid failed!");
        return -2;
    }

    listener = env->GetObjectField(java_pdr_lib_obj, listener_fid);
    if (0 == listener) {
        // LOGE("GetObjectField listener is 0, do you forget setting listener ?");
        return -3;
    }
    pdr_lib->listener_java = env->NewGlobalRef(listener);

    java_pdr_lib_cls = env->GetObjectClass(listener);
    pdr_lib->OnEvent = env->GetMethodID(java_pdr_lib_cls, CB_PDR_DATA_NAME, CB_PDR_DATA_DESC);
    if (0 == pdr_lib->OnEvent) {
        // LOGE("OnStepEvent method not found!!!");
        return -4;
    }

    env->DeleteLocalRef(listener);
    env->DeleteLocalRef(java_pdr_lib_cls);

    return 0;
}

int setJNIPressureCallback(JNIEnv *env, jobject java_pdr_lib_obj) {

    jclass java_pdr_lib_cls;
    jfieldID listener_fid;
    jobject listener;

    floor_lib->env = env;
    floor_lib->lib_java = env->NewGlobalRef(java_pdr_lib_obj);
    if (0 == floor_lib->lib_java) {
        // LOGE("pdr_lib NewGlobalRef failed!");
        return -1;
    }

    java_pdr_lib_cls = env->GetObjectClass(java_pdr_lib_obj);
    listener_fid = env->GetFieldID(java_pdr_lib_cls, LISTENER_PRESSURE_FIELD_ID, LISTENER_PRESSURE_JAVA_PATH);
    if (0 == listener_fid) {
        // LOGE("GetFieldID listener_fid failed!");
        return -2;
    }

    listener = env->GetObjectField(java_pdr_lib_obj, listener_fid);
    if (0 == listener) {
        // LOGE("GetObjectField listener is 0, do you forget setting listener ?");
        return -3;
    }
    floor_lib->listener_java = env->NewGlobalRef(listener);

    java_pdr_lib_cls = env->GetObjectClass(listener);
    floor_lib->OnEvent = env->GetMethodID(java_pdr_lib_cls, CB_PRESSURE_DATA_NAME, CB_PRESSURE_DATA_DESC);
    if (0 == floor_lib->OnEvent) {
        // LOGE("OnStepEvent method not found!!!");
        return -4;
    }

    env->DeleteLocalRef(listener);
    env->DeleteLocalRef(java_pdr_lib_cls);

    return 0;
}

static void OnPdrStepCallbackEvent(double  x, double y, double sl, double yaw ,double deta_angle) {
    if (0 != pdr_lib->listener_java) {
        if (step != NULL)
            pdr_lib->env->CallVoidMethod(pdr_lib->listener_java, pdr_lib->OnEvent,
                                        (jdouble) x,
                                        (jdouble) y,
                                        (jdouble) sl,
                                        (jdouble) yaw,
					                    (jdouble) -deta_angle);
    }
}

static void OnFloorChangeCallbackEvent(int floorType, int upOrDown, double k, double a ,int current_floorIndex){
    if (0 != floor_lib->listener_java) {
        if (step != NULL)
            floor_lib->env->CallVoidMethod(floor_lib->listener_java, floor_lib->OnEvent,
                                        (jint) floorType,
                                        (jint) upOrDown,
                                        (jdouble) k,
                                        (jdouble) a,
					                    (jint) current_floorIndex);
    }  
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_initJniPdrStep(JNIEnv *env, jclass) {

    if(NULL == vm) {
        env->GetJavaVM(&vm);
    }

    if(NULL == pdr_lib) {
        pdr_lib = new JniPdrStepLib();
    }

    if(NULL == floor_lib) {
        floor_lib = new JniPdrStepLib();
    }

    if(NULL == step) {
        step = new SHS::PDRSIM();
    }

    step->setCallBack(OnPdrStepCallbackEvent);
    step->setFloorCallBack(OnFloorChangeCallbackEvent);
}

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setJniPdrStepCallback(JNIEnv *env, jclass, jobject object) {

    if(NULL != vm) {
        JNIEnv *env2 = NULL;
        if(0 == vm->AttachCurrentThread(&env2, NULL) ) {
            return setJNIPdrStepCallback(env2, object);
        }
    }

    return setJNIPdrStepCallback(env, object);
}

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setJniPressureCallback(JNIEnv *env, jclass, jobject object) {

    if(NULL != vm) {
        JNIEnv *env2 = NULL;
        if(0 == vm->AttachCurrentThread(&env2, NULL) ) {
            return setJNIPressureCallback(env2, object);
        }
    }

    return setJNIPressureCallback(env, object);
}

JNIEXPORT jboolean JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_nextStepEx(JNIEnv *env, jclass, jdouble gyro_x,jdouble gyro_y,jdouble gyro_z,jdouble linear_acc_x,jdouble linear_acc_y,jdouble linear_acc_z,jdouble gx,jdouble gy,jdouble gz,jdouble mag_x,jdouble mag_y,jdouble mag_z,jdouble time_r)
{
    jboolean res = false;

    if(step != NULL) {
        res = step->adddata(gyro_x, gyro_y, gyro_z, linear_acc_x, linear_acc_y, linear_acc_z,gx,gy,gz, mag_x, mag_y, mag_z, time_r);
    }

    return res;
}

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_releaseJniPdrLib(JNIEnv *env, jclass) {
    if (step != NULL) {
        delete step;
        step = NULL;
    }
    if (pdr_lib != NULL) {
        delete pdr_lib;
        pdr_lib = NULL;
    }
    if(floor_lib != NULL) {
        delete floor_lib;
        floor_lib = NULL;
    }
    return 0;
}

JNIEXPORT jstring JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getVersionFromJNI(JNIEnv *env, jclass) {
//2.1增加了气压的部分
    std::string hello = "Version 2.1";
    return env->NewStringUTF(hello.c_str());
}
JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getX(JNIEnv *, jclass){
    return step->get_X();
}

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getY(JNIEnv *, jclass){
    return step->get_Y();
}

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getSL(JNIEnv *, jclass){
    return step->get_SL();
}

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getYAW(JNIEnv *, jclass){
    return step->get_YAW();
}

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getDetaAngle(JNIEnv *, jclass){
  return step->get_deta_angle();
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setMagOffset(JNIEnv *, jclass ,jdouble offset)
{
  
  step->set_magoffset(offset);
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setXY(JNIEnv *, jclass ,jdouble x,jdouble y)
{
  step->setXY(x,y);
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setXYYaw(JNIEnv*, jclass, jdouble x, jdouble y, jdouble yaw)
{
  step->setXY(x,y);
  step->setYaw(yaw);
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setYaw(JNIEnv*, jclass, jdouble yaw)
{
  step->setYaw(yaw);
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitXYYaw(JNIEnv *, jclass ,jdouble x,jdouble y,jdouble yaw)
{
  step->InitialXYYaw(x,y,yaw);
  
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitXY(JNIEnv *, jclass ,jdouble x,jdouble y)
{
  step->InitialXY(x,y);
  
}
JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_chooseMethod(JNIEnv *, jclass ,jint index)
{
  //选择输出，1是陀螺，2是地磁
  step->choose_ahrs(index);
}
JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setStep(JNIEnv*, jclass, jdouble f_scale, jdouble s_scale)
{
  step->stest->set_para(f_scale,s_scale);
}
JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setOutAngle(JNIEnv*, jclass, jint index)
{
  //0:yaw,1:积分，2：夹角
  step->set_OutAngle(index);
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitFloor(JNIEnv*, jclass, jint curfloorIndex)
{
  step->InitFloorModule(curfloorIndex);
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_InitXYYawFloor(JNIEnv*, jclass, jdouble x, jdouble y,jdouble yaw, jint curfloorIndex)
{
  step->InitialXYYawFloor(x,y,yaw,curfloorIndex);
}
JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_floorModuleAddData(JNIEnv*, jclass, jdouble acc_norm, jdouble orientation, jdouble pressure, jdouble time_r)
{
  
   jint res = false;

    if(step != NULL) {
        res = step->floorModuleAddData(acc_norm,orientation,pressure,time_r);
    }

    return res;
}
JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_floorModuleAddFloorHeight(JNIEnv*, jclass, jdouble floor_height)
{
  step->floorModuleAddFloorHeight(floor_height);
}
JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_floorModuleUpdateFloorHeightMatrix(JNIEnv*, jclass)
{
  step->floorModuleUpdateFloorHeightMatrix();
}

JNIEXPORT void JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setPdrEkf(JNIEnv*, jclass)
{
  step->pdrEkf=true;
}

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_setCurrentFloorIndex(JNIEnv*, jclass,jint floorId)
{
  return step->floortest->setCurrentFloorIndex(floorId);
}

JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getCurrentFloorIndex(JNIEnv*, jclass)
{
  return step->floortest->getCurrentFloorIndex();
}

JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getFloorAccStd(JNIEnv*, jclass)
{
  return step->floortest->getAccStd();
}
JNIEXPORT jdouble JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getFloorK(JNIEnv*, jclass)
{
  return step->floortest->getK();
}
JNIEXPORT jint JNICALL Java_com_xihe_newpdrsdk_locator_PdrStep_getFloorType(JNIEnv *, jclass)
{
  return step->floortest->getFloorType();
}
