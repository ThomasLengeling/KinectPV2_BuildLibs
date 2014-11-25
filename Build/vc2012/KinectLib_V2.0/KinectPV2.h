/*
KinectV2.0 library for Processing
Copyright (c) 2014 Thomas Sanchez Lengeling

* Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following
* conditions are met:st
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
*

KinectfV2.0 library  library for Processing is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

KinectfV2.0 library for Processing is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with KinectfV2.0 library for Processing.  If not, see
<http://www.gnu.org/licenses/>.
*/

#pragma once

#include <string>
#include <stdint.h>
#include <exception>
#include <comutil.h>
#include <iostream>
#include <memory>
#include <vector>

#include "ole2.h"


#include "Kinect.h"
#include "Kinect.Face.h"

#define M_PI 3.14159265358979323846

#define VERSION			"0.7.0"

static const int         cColorWidth = 1920;
static const int         cColorHeight = 1080;

static const int         cDepthWidth = 512;
static const int         cDepthHeight = 424;

#define frame_size_color 2073600
#define frame_size_depth 217088

static const int BUFFER_SIZE_COLOR = frame_size_color * 4;

//SKELETON
static const int JOINTSIZE = BODY_COUNT * (JointType_Count + 1) * 9;

static const int HDFACEVERTEX = BODY_COUNT * 1347 * 2 + BODY_COUNT;

//FACE DETECTION
static const int FACESIZE = BODY_COUNT * (36);

//////////////////////////////////////////////////////////////////////////////////////////////
namespace KinectPV2{
	class DeviceOptions{

	public:
		DeviceOptions();
		~DeviceOptions(){}

		void			enableColorImage(bool toggle = true){ toggleColorFrame = toggle; }
		void			enableDepthImage(bool toggle = true){ toggleDepthFrame = toggle; }
		void			enableDepthMaskImage(bool toggle = true){
								toggleDepthMaskFrame = toggle;
								if (toggleDepthMaskFrame){
									toggleDepthFrame = true;
								}
		}

		void			enableFaceDetection(bool toggle = false){
								toggleFaceDetection = toggle;
								if (toggleFaceDetection){
									toggleSkeleton = true;
									toggleColorFrame = true;
									toggleInFraredFrame = true;
								}
		}

		void			enableHDFaceDetection(bool toggle = false){
								toggleHDFaceDetection = toggle;
								if (toggleHDFaceDetection){
									toggleSkeleton = true;
									toggleColorFrame = true;
									toggleInFraredFrame = true;
									//toggleBodyTrack = true;
								}
		}

		void			enableCoordinateMapperColor(bool toggle = false){
								togglCoodinateMappingColor = toggle;
								if (togglCoodinateMappingColor){
									toggleColorFrame = true;
									toggleDepthFrame = true;
									toggleBodyTrack  = true;
								}
						}

		void            enableInFraredImage(bool toggle = true){ toggleInFraredFrame = toggle; }
		void			enableInFraredExposureImage(bool toggle = true){ toggleInFraredLongExposure = toggle; }


		void            enableBodyTrack(bool toggle = true){ toggleBodyTrack = toggle; }

		void			enablePointCloud(bool toggle = true){ togglePointCloud = toggle; }
		void			enablePointCloudColor(bool toggle = true){ togglePointCloudColor = toggle; }

		void			enableRawDepthData(bool toggle = true){ toggleRawDepthData = toggle; }

		void            enableSkeleton(bool toggle = true){ toggleSkeleton = toggle; }
		void			enableSkeletonDepthMap(bool toggle = true){ 
									toggleSkeletonDepthMap = toggle;
									if (toggleSkeletonDepthMap){
										toggleDepthFrame = true;
									}
						}

		void			enableSkeleton3dMap(bool toggle = true){ toggleSkeleton3dMap = toggle; }
		void			enableSkeletonColorMap(bool toggle = true){
									toggleSkeletonColorMap = toggle;
									if (toggleSkeletonColorMap){
										toggleColorFrame = true;
									}
						}

		inline bool		isEnableColorFrame(){ return toggleColorFrame; }
		inline bool     isEnableDepthFrame(){ return toggleDepthFrame; }
		inline bool		isEnableDepthMaskFrame(){ return toggleDepthMaskFrame; }
		inline bool		isEnableInFraredFrame(){ return toggleInFraredFrame; }
		inline bool		isEnableInfraredExposureFrame(){ return toggleInFraredLongExposure; }

		inline bool     isEnableFaceDetection(){ return toggleFaceDetection; }
		inline bool		isEnableHDFaceDetection(){ return toggleHDFaceDetection; }

		inline bool		isEnableBodyTrack(){ return toggleBodyTrack; }

		inline bool		isEnablePointCloud(){ return togglePointCloud; }
		inline bool		isEnablePointCloudColor(){ return togglePointCloudColor; }

		inline bool		isEnableRawDepthData(){ return toggleRawDepthData; }

		inline bool     isEnableSkeleton(){ return toggleSkeleton; }
		inline bool		isEnableSkeletonDepthMap(){ return toggleSkeletonDepthMap; }
		inline bool		isEnableSkeleton3dMap(){ return toggleSkeleton3dMap; }
		inline bool		isEnableSkeletonColorMap(){ return toggleSkeletonColorMap; }

		inline bool		isEnableCoordinateMappingColor(){ return togglCoodinateMappingColor;}

	private:
		bool		toggleColorFrame;
		bool		toggleDepthFrame;
		bool		toggleDepthMaskFrame;

		bool		toggleFaceDetection;
		bool		toggleHDFaceDetection;

		bool		toggleInFraredFrame;
		bool        toggleInFraredLongExposure;
	
		bool		togglePointCloud;
		bool		togglePointCloudColor;
		
		bool		toggleRawDepthData;

		bool        toggleBodyTrack;
		bool		toggleSkeleton;

		bool		toggleSkeletonDepthMap;
		bool		toggleSkeleton3dMap;
		bool		toggleSkeletonColorMap;

		bool		togglCoodinateMappingColor;
	};

	class Device : public DeviceOptions
	{
	private:
		uint8_t *	 pixelsData;
		uint8_t *    colorFrameData;

		uint8_t	*	 outCoordMapperRGBX;

		
		uint32_t *	 depthData;
		uint32_t *   depthMaskData;
		uint32_t *   depthRawData;

		float    *	 pointCloudPosData;
		float    *   pointCloudColorData;

		uint32_t *   pointCloudDepthImage;
		float    *   pointCloudDepthNormalized;

		float	 *   pointCloudRawImage;

		float	 *   colorCameraPos;

		uint32_t *	 infraredData;
		uint32_t *	 longExposureData;

		float    *   skeletonData3dMap;
		float    *   skeletonDataDepthMap;
		float    *   skeletonDataColorMap;

		float	 *   hdFaceDeformations;
		float    *   hdFaceVertex;

		UINT32		 hdFaceVertexCount;

		float	 *   faceData;

		uint32_t *   bodyTrackData;

		int			appWidth;
		int         appHeight;

		bool		mirror;

	protected:

		IKinectSensor					*	kSensor;

		IMultiSourceFrameReader			*	kMultiSourceFrameReader;

		ICoordinateMapper	            *	kCoordinateMapper;
		//IBodyFrameReader				*   kBodyFrameReader;

		// FACE READER
		IFaceFrameSource				*	kFaceFrameSources[BODY_COUNT];
		IFaceFrameReader				*	kFaceFrameReaders[BODY_COUNT];

		//HD FACE READER
		IHighDefinitionFaceFrameSource  *	kHDFaceSource[BODY_COUNT];
		IHighDefinitionFaceFrameReader  *   kHDFaceReader[BODY_COUNT];
		IFaceAlignment					*   kFaceAlignment[BODY_COUNT];
		IFaceModel						*   kFaceModel[BODY_COUNT];
		IFaceModelBuilder				*   kFaceModelBuilder[BODY_COUNT];
		bool								produce[BODY_COUNT];

		CameraSpacePoint				*   mCamaraSpacePointDepth;
		CameraSpacePoint				*   mCamaraSpacePointColor;
		ColorSpacePoint					*	mColorSpacePoint;
		DepthSpacePoint					*   mDepthCoordinates;

		float						depthPCLowTh;
		float						depthPCHighTh;

	public:
		Device(void);
		~Device(void);

		bool	init();
		void	stop();
		bool	update();

		void	enableMirror(bool enableMirror){ mirror = enableMirror;}

		void	setWindowSize(int appWidth, int appHeight);



		uint8_t *	 backgroundRGBX;

		//-----JNI
		uint8_t  *						JNI_GetImage();
		uint32_t *						JNI_GetDepth();
		uint32_t *						JNI_GetDepthRawData();
		uint32_t *						JNI_GetDepthSha();
		uint32_t *						JNI_GetInfrared();
		uint32_t *						JNI_GetLongExposureInfrared();

		uint8_t  *						JNI_GetCoodinateRGBX();

		uint32_t *						JNI_GetDepthMask();

		uint32_t *						JNI_GetBodyTrack();

		//FACE
		float *						    JNI_getFaceRawData();
		float *							JNI_getHDFaceVertexRawData();

		float *							JNI_pointCloudPosData();

		uint32_t   *					JNI_pointCloudDepthImage();
		float      *					JNI_pointCloudDepthNormalized();

		float *							JNI_pointCloudColorData();

		float *							JNI_getSkeletonDataDepthMap();
		float *							JNI_getSkeletonData3dMap();
		float *							JNI_getSkeletonDataColorMap();

		std::string						JNI_version() { return VERSION; }


		void							setSkeletonType(int val){ skeletonMapType = val; }

		//HELP FUNTIONS
		float *							BodyToScreenColor(const CameraSpacePoint& bodyPoint);
		float *							BodyToScreenDepth(const CameraSpacePoint& bodyPoint);

		int								colorByte2Int(int gray);
		int								colorFloat2Int(float gray);
		HRESULT							UpdateBodyData(IBody** ppBodies, IMultiSourceFrame* frame);
		static void						ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll);
		float							lmap(float val, float inMin, float inMax, float outMin, float outMax);

		float							constrain(float val, float min, float max);

		bool							infraredFrameReady;
		bool							colorFrameReady;
		bool							depthFrameReady;
		bool							bodyIndexReady;
		bool							longExposureReady;
		bool							depthMaskReady;

		bool							depthPointCloudFrameReady;
		bool							depthPointCloudImageReady;
		bool							colorPointCloudFrameReady;

		bool							skeleton3dReady;
		bool							skeletonDepthReady;
		bool							skeletonColorReady;

		int								skeletonMapType;
		
		//FACE
		bool							faceDetectionReady;
		bool							hdFaceDetectionReady;

		bool							coordinateRGBXReady;

		void							setLowThresholdDepthPC(float val){ depthPCLowTh = val; }
		float							getLowThresholdDepthPC(){ return depthPCLowTh; }


		void							setHighThresholdDepthPC(float val){ depthPCHighTh = val; }
		float							getHighThresholdDepthPC(){ return depthPCHighTh; }
		//
		//uint32_t						depthJNI[frame_size_depth];
	};

}