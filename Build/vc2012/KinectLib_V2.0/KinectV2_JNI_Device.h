/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
#include "KinectPV2.h"
/* Header for class KinectPV2_Device */

#ifndef _Included_KinectPV2_Device
#define _Included_KinectPV2_Device
#ifdef __cplusplus
extern "C" {
#endif
	/*
	* Class:     KinectPV2_Device
	* Method:    jniDevice
	* Signature: ()V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniDevice
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniInit
	* Signature: ()Z
	*/
	JNIEXPORT jboolean JNICALL Java_KinectPV2_Device_jniInit
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniVersion
	* Signature: ()Ljava/lang/String;
	*/
	JNIEXPORT jstring JNICALL Java_KinectPV2_Device_jniVersion
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniUpdate
	* Signature: ()Z
	*/
	JNIEXPORT jboolean JNICALL Java_KinectPV2_Device_jniUpdate
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniStopDevice
	* Signature: ()V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniStopDevice
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableColorFrame
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableColorFrame
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableColorChannelsFrame
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableColorChannelsFrame
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableDepthFrame
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableDepthFrame
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableDepthMaskFrame
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableDepthMaskFrame
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableInfraredFrame
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableInfraredFrame
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableBodyTrackFrame
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableBodyTrackFrame
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableLongExposureInfrared
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableLongExposureInfrared
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableSkeleton
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableSkeleton
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniSendCoordinateBkg
	* Signature: ([I)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSendCoordinateBkg
		(JNIEnv *, jobject, jintArray);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniSendCoordinateDepth
	* Signature: ([I)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSendCoordinateDepth
		(JNIEnv *, jobject, jintArray);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniSetNumberOfUsers
	* Signature: (I)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSetNumberOfUsers
		(JNIEnv *, jobject, jint);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableFaceDetection
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableFaceDetection
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableHDFaceDetection
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableHDFaceDetection
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableCoordinateMappingRGBDepth
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableCoordinateMappingRGBDepth
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnablePointCloud
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnablePointCloud
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnablePointCloudColor
	* Signature: (Z)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnablePointCloudColor
		(JNIEnv *, jobject, jboolean);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniSetLowThresholdDepthPC
	* Signature: (F)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSetLowThresholdDepthPC
		(JNIEnv *, jobject, jfloat);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniGetLowThresholdDepthPC
	* Signature: ()F
	*/
	JNIEXPORT jfloat JNICALL Java_KinectPV2_Device_jniGetLowThresholdDepthPC
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniSetHighThresholdDepthPC
	* Signature: (F)V
	*/
	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSetHighThresholdDepthPC
		(JNIEnv *, jobject, jfloat);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniGetHighThresholdDepthPC
	* Signature: ()F
	*/
	JNIEXPORT jfloat JNICALL Java_KinectPV2_Device_jniGetHighThresholdDepthPC
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableMapDethCamaraTable
	* Signature: ()[F
	*/
	JNIEXPORT jfloatArray JNICALL Java_KinectPV2_Device_jniEnableMapDethCamaraTable
		(JNIEnv *, jobject);

	/*
	* Class:     KinectPV2_Device
	* Method:    jniEnableMapDethToColorSpace
	* Signature: ()[F
	*/
	JNIEXPORT jfloatArray JNICALL Java_KinectPV2_Device_jniEnableMapDethToColorSpace
		(JNIEnv *, jobject);


	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniColorReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniDepthReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniInfraredReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniBodyIndexReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniInfraredLongExposureReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSkeletonReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniDepthMaskBodyIndexReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniFaceDetectionReadyCopy
		(JNIEnv *, jobject, jboolean);


	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniHDFaceVertexReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniPointColorReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL Java_KinectPV2_Device_jniDepthPointCloudImageReadyCopy
		(JNIEnv *, jobject, jboolean);


	JNIEXPORT void JNICALL  Java_KinectPV2_Device_jniPointCoudPosReadyCopy
		(JNIEnv *, jobject, jboolean);

	JNIEXPORT void JNICALL  Java_KinectPV2_Device_jniCoordinateMapperReadyCopy
		(JNIEnv *, jobject, jboolean);


	JNIEXPORT jintArray JNICALL  Java_KinectPV2_Device_jniGetBodyIndexUser
		(JNIEnv *, jobject, jint);

#ifdef __cplusplus
}
#endif
#endif
