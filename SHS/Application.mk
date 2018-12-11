#APP_BUILD_SCRIPT :=./Android.mk
#APP_ABI := armeabi
APP_ABI :=  all
#APP_STL := stlport_static
APP_STL := stlport_shared
#APP_OPTIM := debug
# APP_PLATFORM
#NDK_TOOLCHAIN_VERSION := 4.8
#APP_CPPFLAGS += -std=c++0x
# APP_STL的取值：

# system(default)系统默认的C++运行库
# stlport_static以静态链接方式使用的sttport版本的STL
# stlport_shared以动态链接方式使用的sttport版本的STL
# gnustl_static以静态链接方式使用的gnustl版本的STL
# gnustl_shared以动态链接方式使用的gnustl版本的STL
# gabi++_static以静态链接方式使用的gabi++
# gabi++_shared以动态链接方式使用的gabi++
# c++_static以静态链接方式使用的LLVM libc++
# c++_shared以动态链接方式使用的LLVM libc++

