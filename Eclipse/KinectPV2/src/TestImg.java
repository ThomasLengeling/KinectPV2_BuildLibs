import java.nio.FloatBuffer;

import KinectPV2.FaceFeatures;
import KinectPV2.KJoint;
import KinectPV2.KinectPV2;
import KinectPV2.Skeleton;
import KinectPV2.FaceData;
import KinectPV2.HDFaceData;
import KinectPV2.KRectangle;
import processing.core.*;
import processing.opengl.PGL;
import processing.opengl.PJOGL;

import javax.media.opengl.GL2;

public class TestImg extends PApplet {

	private KinectPV2 kinect;

	Skeleton [] skeleton;
	
	FaceData [] faceData;
	
	
	public void setup() {
		size(512*3, 424*2, P3D);
		
		
		kinect = new KinectPV2(this);
		//kinect.enableCoordinateMapperRGBDepth(true);
		kinect.enableColorImg(true);
		kinect.enableDepthImg(true);
		kinect.enableInfraredImg(true);
		kinect.enableInfraredLongExposureImg(true);
		kinect.enableBodyTrackImg(true);
		kinect.enableDepthMaskImg(true);
		
	//	kinect.enableLongExposureInfraredImg(true);
	//	
		//kinect.enableInfraredImg(true);
		//kinect.enableBodyTrackImg(true);
	//
		//kinect.enablePointCloud(true);
		
		kinect.init();
		//kinect.setNumberOfUsers(1);
	}
	
	public void draw() {
		background(0);
		
		image(kinect.getColorImage(), 0, 424, 512, 424);
		image(kinect.getDepthImage(), 0, 0, 512, 424);
		
		image(kinect.getInfraredImage(), 512, 0, 512, 424);
		image(kinect.getInfraredLongExposureImage(), 512, 424, 512, 424);
		
		image(kinect.getBodyTrackImage(), 512*2, 0, 512,  424);
		image(kinect.getDepthMaskImage(), 512*2, 424, 512,  424);
		
		//image(kinect.getLongExposureInfraredImage(), 0, 0);
		//image(kinect.getBodyTrackImage(), 0, 0);
		
		  fill(255, 0, 0);
		  text(frameRate, 50, 50);
	}
}