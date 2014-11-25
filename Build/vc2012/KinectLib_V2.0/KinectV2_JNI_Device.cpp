#include "stdafx.h"
#include "KinectV2_JNI_Device.h"


JNIEXPORT void JNICALL Java_KinectPV2_Device_jniDevice
(JNIEnv * env, jobject obj)
{
	KinectPV2::Device* kinect = new KinectPV2::Device();

	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	env->SetLongField(obj, fid, (jlong)kinect);
	env->DeleteLocalRef(cls);
}

JNIEXPORT jboolean JNICALL Java_KinectPV2_Device_jniInit
(JNIEnv * env, jobject obj)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	jboolean result = (jboolean)kinect->init();
	env->DeleteLocalRef(cls);
	return result;
}


JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSetWindowSizeSkeleton
(JNIEnv * env, jobject obj, jint width, jint height)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->setWindowSize((int)width, (int)height);
	env->DeleteLocalRef(cls);
}

JNIEXPORT jstring JNICALL Java_KinectPV2_Device_jniVersion
(JNIEnv * env, jobject obj)
{
	jstring result;

	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);

	std::string version = kinect->JNI_version();
	result = env->NewStringUTF(version.c_str());
	env->DeleteLocalRef(cls);
	return  result;
}


JNIEXPORT jboolean JNICALL Java_KinectPV2_Device_jniUpdate
(JNIEnv * env, jobject obj)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	bool result = kinect->update();

	if (result == true){
		if (kinect->colorFrameReady){
			jint * pInt = (jint *)kinect->JNI_GetImage();
			
			jintArray buffer = env->NewIntArray(frame_size_color);
			env->SetIntArrayRegion(buffer, 0, frame_size_color, (jint *)(pInt));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyColorImg", "([I)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->colorFrameReady = false;
		}
		if (kinect->depthFrameReady && kinect->isEnableDepthFrame()){
			jint * pInt = (jint *)kinect->JNI_GetDepth();

			jintArray buffer = env->NewIntArray(frame_size_depth);
			env->SetIntArrayRegion(buffer, 0, frame_size_depth, pInt);

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyDepthImg", "([I)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->depthFrameReady = false;
		}

		if (kinect->depthPointCloudFrameReady){
			jfloat * pFloat = (jfloat *)kinect->JNI_pointCloudPosData();

			jfloatArray buffer = env->NewFloatArray(frame_size_depth * 3);
			env->SetFloatArrayRegion(buffer, 0, frame_size_depth * 3, pFloat);

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyPointCloudPos", "([F)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->depthPointCloudFrameReady = false;

		}
		if (kinect->depthPointCloudImageReady){
			jint * pFloatCloud = (jint *)kinect->JNI_pointCloudDepthImage();

			jintArray bufferCloud = env->NewIntArray(frame_size_depth);
			env->SetIntArrayRegion(bufferCloud, 0, frame_size_depth, pFloatCloud);

			jclass clazzCloud       = env->GetObjectClass(obj);
			jmethodID methodIDCloud = env->GetMethodID(clazzCloud, "copyPointCloudImage", "([I)V");
			env->CallVoidMethod(obj, methodIDCloud, bufferCloud);

			kinect->depthPointCloudImageReady = false;
		}

		if (kinect->colorPointCloudFrameReady){
			jfloat * pFloat = (jfloat *)kinect->JNI_pointCloudColorData();

			jfloatArray buffer = env->NewFloatArray(frame_size_color * 3);
			env->SetFloatArrayRegion(buffer, 0, frame_size_color * 3, pFloat);

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyPointCloudColor", "([F)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->colorPointCloudFrameReady = false;
		}
		if (kinect->depthMaskReady){
			jint * pInt = (jint *)kinect->JNI_GetDepthMask();

			jintArray buffer = env->NewIntArray(frame_size_depth);
			env->SetIntArrayRegion(buffer, 0, frame_size_depth, (jint *)(pInt));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyDepthMaskImg", "([I)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->depthMaskReady = false;
		}

		if (kinect->bodyIndexReady){
			jint * pInt = (jint *)kinect->JNI_GetBodyTrack();

			jintArray buffer = env->NewIntArray(frame_size_depth);
			env->SetIntArrayRegion(buffer, 0, frame_size_depth, (jint *)(pInt));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyBodyTrackImg", "([I)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->bodyIndexReady = false;
		}


		if (kinect->infraredFrameReady){
			jint * pInt = (jint *)kinect->JNI_GetInfrared();

			jintArray buffer = env->NewIntArray(frame_size_depth);
			env->SetIntArrayRegion(buffer, 0, frame_size_depth, (jint *)(pInt));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyInfraredImg", "([I)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->infraredFrameReady = false;
		}

		if (kinect->longExposureReady){
			jint * pInt = (jint *)kinect->JNI_GetLongExposureInfrared();

			jintArray buffer = env->NewIntArray(frame_size_depth);
			env->SetIntArrayRegion(buffer, 0, frame_size_depth, (jint *)(pInt));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyLongExposureImg", "([I)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->longExposureReady = false;
		}

		if (kinect->skeletonDepthReady){
			jfloat * pfloat = (jfloat *)kinect->JNI_getSkeletonDataDepthMap();

			jfloatArray buffer = env->NewFloatArray(JOINTSIZE);
			env->SetFloatArrayRegion(buffer, 0, JOINTSIZE, (jfloat *)(pfloat));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copySkeletonDepthData", "([F)V");
			env->CallVoidMethod(obj, methodID, buffer);
			kinect->skeletonDepthReady = false;
		}

		if (kinect->skeletonColorReady){
			jfloat * pfloat = (jfloat *)kinect->JNI_getSkeletonDataColorMap();

			jfloatArray buffer = env->NewFloatArray(JOINTSIZE);
			env->SetFloatArrayRegion(buffer, 0, JOINTSIZE, (jfloat *)(pfloat));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copySkeletonColorData", "([F)V");
			env->CallVoidMethod(obj, methodID, buffer);
			kinect->skeletonColorReady = false;
		}

		if (kinect->skeleton3dReady){
			jfloat * pfloat = (jfloat *)kinect->JNI_getSkeletonData3dMap();

			jfloatArray buffer = env->NewFloatArray(JOINTSIZE);
			env->SetFloatArrayRegion(buffer, 0, JOINTSIZE, (jfloat *)(pfloat));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copySkeleton3DData", "([F)V");
			env->CallVoidMethod(obj, methodID, buffer);
			kinect->skeleton3dReady = false;
		}

		if (kinect->faceDetectionReady){
			jfloat * pfloat = (jfloat *)kinect->JNI_getFaceRawData();

			jfloatArray buffer = env->NewFloatArray(FACESIZE);
			env->SetFloatArrayRegion(buffer, 0, FACESIZE, (jfloat *)(pfloat));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyFaceRawData", "([F)V");
			env->CallVoidMethod(obj, methodID, buffer);
			kinect->faceDetectionReady = false;
		}

		if (kinect->coordinateRGBXReady){
			jint * pInt = (jint *)kinect->JNI_GetCoodinateRGBX();

			jintArray buffer = env->NewIntArray(frame_size_color);
			env->SetIntArrayRegion(buffer, 0, frame_size_color, (jint *)(pInt));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyCoordinateMapper", "([I)V");
			env->CallVoidMethod(obj, methodID, buffer);
			kinect->coordinateRGBXReady = false;
		}
		if (kinect->hdFaceDetectionReady){

			jfloat * pfloat = (jfloat *)kinect->JNI_getHDFaceVertexRawData();

			jfloatArray buffer = env->NewFloatArray(HDFACEVERTEX);
			env->SetFloatArrayRegion(buffer, 0, HDFACEVERTEX, (jfloat *)(pfloat));

			jclass clazz = env->GetObjectClass(obj);
			jmethodID methodID = env->GetMethodID(clazz, "copyHDFaceVertexRawData", "([F)V");
			env->CallVoidMethod(obj, methodID, buffer);

			kinect->hdFaceDetectionReady = false;
		}

	}

	env->DeleteLocalRef(cls);
	env->DeleteLocalRef(obj);

	return (jboolean)result;
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniStopDevice
(JNIEnv * env, jobject obj)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->stop();
	env->DeleteLocalRef(cls);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableColorFrame
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableColorImage((bool)toggle);
	env->DeleteLocalRef(cls);
}
JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableDepthFrame
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableDepthImage((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableDepthMaskFrame
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableDepthMaskImage((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableInfraredFrame
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableInFraredImage((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableBodyTrackFrame
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableBodyTrack((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableLongExposureInfrared
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableInFraredExposureImage((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableSkeleton
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableSkeleton((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableSkeletonDepthMap
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableSkeletonDepthMap((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableSkeleton3dMap
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableSkeleton3dMap((bool)toggle);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableSkeletonColorMap
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableSkeletonColorMap((bool)toggle);
	env->DeleteLocalRef(cls);
}


////FACE DETECTION
JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableFaceDetection
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableFaceDetection((bool)toggle);
	env->DeleteLocalRef(cls);
}

//HDFACE DETECTION
JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableHDFaceDetection
(JNIEnv *env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enablePointCloud((bool)toggle);
	if (toggle == JNI_TRUE)
		kinect->enableHDFaceDetection(true);
	env->DeleteLocalRef(cls);

}

//COORDINATE MAPPER RGB DEPTH
JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnableCoordinateMappingRGBDepth
(JNIEnv *env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enablePointCloud((bool)toggle);
	if (toggle == JNI_TRUE)
		kinect->enableCoordinateMapperColor(true);
	env->DeleteLocalRef(cls);
}

//GET IMAGE
JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSendArrayInts
(JNIEnv *env, jobject obj, jintArray ptr)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);

	int i;
	jsize len = env->GetArrayLength(ptr);
	jint *body = env->GetIntArrayElements(ptr, 0);
	for (i = 0; i < len; i++){
		kinect->backgroundRGBX[i] = body[i];
	}

	env->ReleaseIntArrayElements(ptr, body, 0);
	env->DeleteLocalRef(cls);
}

//POINT CLOUD
JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnablePointCloud
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enablePointCloud((bool)toggle);
	if (toggle == JNI_TRUE)
		kinect->enableDepthImage(true);
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniEnablePointCloudColor
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enablePointCloudColor(bool(toggle));
	env->DeleteLocalRef(cls);
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSetHighThresholdDepthPC
(JNIEnv * env, jobject obj, jfloat val)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->setHighThresholdDepthPC((float)val);
	env->DeleteLocalRef(cls);
}

JNIEXPORT jfloat JNICALL Java_KinectPV2_Device_jniGetHighThresholdDepthPC
(JNIEnv * env, jobject obj)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	float val = kinect->getHighThresholdDepthPC();
	env->DeleteLocalRef(cls);

	return (jfloat)val;
}


JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSetLowThresholdDepthPC
(JNIEnv * env, jobject obj, jfloat val)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->setLowThresholdDepthPC((float)val);
	env->DeleteLocalRef(cls);
}

JNIEXPORT jfloat JNICALL Java_KinectPV2_Device_jniGetLowThresholdDepthPC
(JNIEnv * env, jobject obj)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	float val = kinect->getLowThresholdDepthPC();
	env->DeleteLocalRef(cls);

	return (jfloat)val;
}

JNIEXPORT void JNICALL Java_KinectPV2_Device_jniSetMirror
(JNIEnv * env, jobject obj, jboolean toggle)
{
	jclass cls = env->GetObjectClass(obj);
	jfieldID fid = env->GetFieldID(cls, "ptr", "J");
	KinectPV2::Device * kinect = (KinectPV2::Device *) env->GetLongField(obj, fid);
	kinect->enableMirror(bool(toggle));
	env->DeleteLocalRef(cls);
}
