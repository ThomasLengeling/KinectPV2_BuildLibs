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

public class BodyTrackTest extends PApplet {

	private KinectPV2 kinect;

	Skeleton [] skeleton;
	
	FaceData [] faceData;
	
	
	public void setup() {
		size(512*4, 424*2, P3D);
		
		kinect = new KinectPV2(this);
		kinect.enableBodyTrackImg(true);
		kinect.init();
	}
	
	public void draw() {
		background(255, 0, 0);
	
		image(kinect.getBodyTrackImage(), 0, 424);
		
		kinect.generteBodyTrackUsers();
		
		for(int i = 0; i < 3; i++)
			image(kinect.getBodyTrackUser(i), 512 + 512*i, 0);
		
		for(int i = 0; i < 3; i++)
			image(kinect.getBodyTrackUser(i + 3), 512 + 512*i, 424);
	}
	
	public void keyPressed() {
		if(key == '1') {
			kinect.setNumberOfUsers(1);
			println("1");
		}
		if(key == '2') {
			kinect.setNumberOfUsers(2);
			println("2");
		}
		if(key == '3') {
			kinect.setNumberOfUsers(3);
			println("3");
		}
	}
}