package core;

import java.io.IOException;
import java.io.File;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import jdk.internal.dynalink.linker.LinkerServices;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;
import org.opencv.imgproc.*;
import org.opencv.video.Video;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

class Tuple<X, Y> {
	public final X x;
	public final Y y;
	public Tuple(X x, Y y) {
		this.x = x;
		this.y = y;
	}
}

class Target{
	public RotatedRect rect;
	public MatOfPoint hull;
	public Target(RotatedRect rect, MatOfPoint hull){
		this.rect = rect;
		this.hull = hull;
	}
}

public class centralModule {
	//setup constants
	public static final String ANGLE_SEARCHED = "0";

	public static final int DEBUG_TARGET = 0;
	public static final int TARGET_CROCHET = 1;
	public static final int HIGH_GOAL = 2;
	public static final int GEAR_TERRE = 3;
	public static final int BALLE_TERRE = 4;
	
	public static final float CENTRE_IMAGE_X = 640;
	public static final float CENTRE_IMAGE_Y = 360;
	public static final double IMAGE_WIDTH = 1280.0;
	public static final double IMAGE_HEIGHT = 720.0;
	public static final double FOV_WIDTH = 376.0;
	public static final double CONV_PIXEL2CM = FOV_WIDTH / IMAGE_WIDTH;
	public static final double FOV_LENGHT = 298;
	// setup member variables
	public double m_degree;
	public int m_targetMode;
	public boolean m_targetStop;

	public float currentFPS;

	Scalar m_minHSV;
	Scalar m_maxHSV;

	public double m_exposition;
	public double m_contraste;

	public int m_tailleMin;

	public Logger m_log;
	public FileHandler m_fh;

	VideoCapture m_camera;
	double m_time;
	Mat m_srcImage;
	Mat m_hsvImage;
	Mat m_hsvOverlay;
	double m_timeSinceLastUpdate;

	public double m_targetAngle = 35;
	public Scalar m_targetPosition = new Scalar(0, 0);
	Mat m_originalImage;

	public int cameraPort = 0;
	
	public float m_TargetCenter_X;
	public float m_TargetCenter_Y;
	public float m_imageTargetCenter_X;
	public float m_imageTargetCenter_Y;
	//start of functions

	public centralModule() {
		//Code initialisation

		m_degree = 0;
		m_targetMode = 0;
		m_targetStop = false;

		m_minHSV = new Scalar(0, 0, 0);
		m_maxHSV = new Scalar(255, 255, 255);

		m_exposition = 20;
		m_contraste = 20;

		m_tailleMin = 0;

		createLog();

		//OpenCV initialisation

		cameraPort = 0;
		for(int device = 0; device<10; device++)
		{
			VideoCapture tempCam = new VideoCapture();
			tempCam.open(device);
			if (tempCam.isOpened()){
				m_log.info("Camera is on port "+device);
				cameraPort = device;
				break;
			}
			m_log.info("Camera not on port "+device);
		}

		m_camera = new VideoCapture(cameraPort);
		//m_camera.set(Videoio.CAP_PROP_EXPOSURE, m_contraste);
		//m_camera.open(cameraPort);
		if(!m_camera.isOpened()){
			m_log.severe("Camera Error");
		}
		else{
			m_log.info("Camera OK!");
		}



		String OS = System.getProperty("os.name").toLowerCase();
		if (OS.indexOf("win") >= 0)
			m_camera.open(-1);


		//m_camera.set(Videoio.CAP_PROP_FRAME_COUNT, 30);
		m_camera.set(Videoio.CV_CAP_PROP_FRAME_WIDTH, 640);
		m_camera.set(Videoio.CV_CAP_PROP_FRAME_HEIGHT, 480);

		m_srcImage = new Mat();
		m_camera.read(m_srcImage);


		m_originalImage = Imgcodecs.imread("C:\\Users\\Samy Rasmy\\Documents\\2019-Targets\\10-2.png");
		m_hsvImage = Mat.zeros(m_srcImage.size(), 0);

		m_hsvOverlay = new Mat(3,3,0);

		//m_camera.read(m_hsvOverlay);

		m_time = System.nanoTime();
	}

	public void setTargetMode(int target) {
		m_targetMode = target;
	}

	public int getTargetMode() {
		return m_targetMode;
	}

	public Mat getSrcImage(){
		return m_srcImage;
		//return Imgcodecs.imread("C:\\Users\\Samy Rasmy\\Documents\\2019-Targets\\" + ANGLE_SEARCHED + ".png");
	}

	public void runTarget(boolean liveMode)
	{
		if (liveMode) m_targetStop = true;
		
		m_time = System.nanoTime();
		
		switch (m_targetMode) {
			case (DEBUG_TARGET) : {
				//TODO	Montre l'image et mets le masque HSV
				targetDebug();
				break;
			}
			case (TARGET_CROCHET) : {
				//TODO trouver le centre de deux blob vertical
				targetCrochet();
				break;
			}
			case (HIGH_GOAL): {
				//TODO trouver le centre de deux blob horizontal
				highGoal();
				break;
			}
			case (GEAR_TERRE): {
				//TODO trouver le centre d'un blob par terre
				gearTerre();
				break;
			}
			case (BALLE_TERRE): {
				//TODO trouver le centre d'un blob plus petit
				balleTerre();
				break;
			}
			default : {
				m_log.severe("[runTarget] invalid target mode, change with ''changeTargetMode''");
			}
		}
		
		currentFPS = (float)(1 / ((System.nanoTime() - m_time)/1000000000));
		//m_log.info("FPS = "+currentFPS);

	}

	public void stopTarget () {
		m_targetStop = false;
	}

	public double getTargetResult() {
		return m_degree;
	}

	public void setHSV (int minH,int minS,int minV,int maxH,int maxS,int maxV) { // for masking
		m_minHSV = new Scalar(minH, minS, minV);
		m_minHSV = new Scalar(maxH, maxS, maxV);
	}
	public void setHSV (Scalar min, Scalar max) {
		m_minHSV = min;
		m_maxHSV = max;
	}

	public Scalar getMinHSV(){
		return m_minHSV;
	}
	public Scalar getMaxHSV(){
		return m_maxHSV;
	}

	public void setCamParam (double exposition, double contraste) { // for changing the image found
		m_exposition = exposition;
		m_contraste = contraste;
	}
	public void setExposure(double exposure){
		m_exposition = exposure;
	}
	public void setContrast(double contrast){
		m_contraste = contrast;
	}
	public void setContourParam (int tailleMin) { // for blob finding
		m_tailleMin = tailleMin;
	}


	public int[] getContourParam() {
		int[] contourParam = {m_tailleMin};
		return contourParam;
	}


	//TargetCodes?
	public void targetDebug()
	{
		//This function is ran at every single frame, which means that it floods the log.
		//m_logger.info("[runTarget] targetDebug started");

		//m_camera.set(Videoio.CAP_PROP_EXPOSURE, m_exposition);
		//m_camera.read(m_srcImage);
		//m_srcImage = Imgcodecs.imread("images/frc2017/1.jpg");
		//Utiliser une image disque pour le debogage
		//m_srcImage = Imgcodecs.imread("/home/sysgen/image.JPG");
		//Imgproc.blur(srcImage, srcImage, new Size(3, 3));
		
		/*
		 *  Convertis une image de BGR de la camera a HSV
		 *  
		 *  Raison: HSV permet de choisir une certaine couleur de cible
		 */
		Imgproc.cvtColor(m_srcImage, m_hsvImage, Imgproc.COLOR_BGR2HSV);
		
		/*
		 * Permet de faire le choix de l'intervale de couleur
		 * 
		 * Param1 = source, Param2 = 3 valeur minimum HSV, Param3 = 3 valeur maximum HSV, Param 4 = destination
		 *
		 */
		Core.inRange(m_hsvImage, m_minHSV , m_maxHSV, m_hsvOverlay); // Valeur pour le tape

		//Core.multiply(m_hsvOverlay, new Scalar(0.75, 0.75, 0.75), m_hsvOverlay);
		//Core.multiply(m_hsvOverlay, new Scalar(0.3, 1, 1), m_hsvOverlay);

		//Nous allons utiliser le maintenant inutile hsvImage comme Mat de swap...
		Imgproc.cvtColor(m_hsvOverlay, m_hsvImage, Imgproc.COLOR_GRAY2BGR);
		
		
		/*
		 * findContour va trouver les contours des objets de l'image
		 * 
		 */
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(m_hsvOverlay, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		//Core.multiply(srcImage, new Scalar(0,0,0), srcImage);

		//Appliquer le masque...

		//Imgproc.cvtColor(m_hsvOverlay, m_hsvOverlay, Imgproc.COLOR_GRAY2BGR);
		//Core.bitwise_and(srcImage, m_hsvOverlay, srcImage);

		List<MatOfInt> convexhulls = new ArrayList<MatOfInt>(contours.size());
		List<Double> orientations = new ArrayList<Double>();
//		//Dessiner les rectangles
		for (int i = 0; i < contours.size(); i++)
		{
//			//Trier les contours qui ont une bounding box
			convexhulls.add(i, new MatOfInt(6));
			if (Imgproc.contourArea(contours.get(i)) > 1000)
			{
				Imgproc.convexHull(contours.get(i), convexhulls.get(i));
				//double contourSolidity = Imgproc.contourArea(contours.get(i))/Imgproc.contourArea(convexhulls.get(i));
				Imgproc.drawContours(m_srcImage, contours, i, new Scalar(255, 255, 255), -1);
				MatOfPoint2f points = new MatOfPoint2f(contours.get(i).toArray());
				RotatedRect rRect = Imgproc.minAreaRect(points);

				Point[] vertices = new Point[4];
				rRect.points(vertices);
				for (int j = 0; j < 4; j++) //Dessiner un rectangle avec rotation..
				{
					Imgproc.line(m_srcImage, vertices[j], vertices[(j+1)%4], new Scalar(0,255,0), 10);
				}
				//System.out.println(contourSolidity);
			}
		}
		//m_srcImage = m_hsvOverlay;
		//this is a test code
		m_targetAngle = 0;
	}

	public float getFPS(){
		return currentFPS;
	}

	public void targetCrochet() {
//This function is ran at every single frame, which means that it floods the log.
		//m_logger.info("[runTarget] targetDebug started");

		//m_camera.set(Videoio.CAP_PROP_EXPOSURE, m_exposition);
		//m_camera.set(Videoio.CAP_PROP_BRIGHTNESS, 00);
		//m_camera.set(Videoio.CAP_PROP_BRIGHTNESS, 1);
		//m_camera.set(Videoio.CAP_PROP_CONTRAST, 50);
		//m_camera.read(m_srcImage);
		//Utiliser une image disque pour le debogage
		//m_srcImage = Imgcodecs.imread("/images/frc2017/1.jpg");
		//Imgproc.blur(srcImage, srcImage, new Size(3, 3));
		
		/*
		 *  Convertis une image de BGR de la camera a HSV
		 *  
		 *  Raison: HSV permet de choisir une certaine couleur de cible
		 */
		while(true) {
			m_camera.read(m_originalImage);
			try {
				Thread.sleep(0);
			}
			catch (java.lang.InterruptedException e){
				e.printStackTrace();
			}
			if (m_originalImage.cols() != 0)
					break;
		}
		m_srcImage = m_originalImage.clone();
		m_hsvImage = m_srcImage.clone();
		Mat backupImage = m_originalImage.clone();
		Imgproc.cvtColor(m_srcImage, m_hsvImage, Imgproc.COLOR_BGR2HSV);
		
		/*
		 * Permet de faire le choix de l'intervale de couleur
		 * 
		 * Param1 = source, Param2 = 3 valeur minimum HSV, Param3 = 3 valeur maximum HSV, Param 4 = destination
		 *
		 */
		Core.inRange(m_hsvImage, m_minHSV , m_maxHSV, m_hsvOverlay); // Valeur pour le tape

		//Core.multiply(m_hsvOverlay, new Scalar(0.75, 0.75, 0.75), m_hsvOverlay);
		//Core.multiply(m_hsvOverlay, new Scalar(0.3, 1, 1), m_hsvOverlay);

		//Nous allons utiliser le maintenant inutile hsvImage comme Mat de swap...
		Imgproc.cvtColor(m_hsvOverlay, m_hsvImage, Imgproc.COLOR_GRAY2BGR);
		
		
		/*
		 * findContour va trouver les contours des objets de l'image
		 * 
		 */
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(m_hsvOverlay, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		//Core.multiply(srcImage, new ScacurrentFPSlar(0,0,0), srcImage);

		//Appliquer le masque...

		//Imgproc.cvtColor(m_hsvOverlay, m_hsvOverlay, Imgproc.COLOR_GRAY2BGR);
		//Core.bitwise_and(srcImage, m_hsvOverlay, srcImage);

		List<MatOfInt> convexhulls = new ArrayList<MatOfInt>(contours.size());
		List<MatOfPoint> targetHulls = new ArrayList<>();
		List<RotatedRect> rRects = new ArrayList<>();
		List<Double> orientations = new ArrayList<Double>();
//		//Dessiner les rectangles
		List<RotatedRect> selectedRect = new ArrayList<>();
		RotatedRect biggestRect = new RotatedRect();
		RotatedRect second = new RotatedRect();
		double biggestArea = 0;
		double secondBiggest = 0;
		Point crochetPos = new Point();
		for (int i = 0; i < contours.size(); i++)
		{
//			//Trier les contours qui ont une bounding box
			convexhulls.add(i, new MatOfInt(6));
			double tempArea = Imgproc.contourArea(contours.get(i));
			if (tempArea > 100)
			{
				Imgproc.convexHull(contours.get(i), convexhulls.get(i));
				//double contourSolidity = Imgproc.contourArea(contours.get(i))/Imgproc.contourArea(convexhulls.get(i));
				//Imgproc.drawContours(m_srcImage, contours, i, new Scalar(255, 255, 255), -1);
				MatOfPoint2f points = new MatOfPoint2f(contours.get(i).toArray());
				RotatedRect rRect = Imgproc.minAreaRect(points);
				MatOfPoint2f approx = new MatOfPoint2f();
				double epsilon = 0.01*Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()),true);
				Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()),approx,epsilon,true);
				double convexArea = Imgproc.contourArea(approx);
				double ratio = rRect.size.area() / convexArea;
				Imgproc.putText(m_srcImage, String.valueOf(ratio), new Point(30, i*70), 1, 1, new Scalar(255, 20, 20));
				Imgproc.putText(m_srcImage, "angle :" + String.valueOf(Math.abs(rRect.angle%90)), rRect.center, 1, 1, new Scalar(155, 120, 20));
				MatOfPoint approxpoints = new MatOfPoint();
				approx.convertTo(approxpoints, CvType.CV_32S);
				List<MatOfPoint> matofapprox = new ArrayList<>();
				matofapprox.add(approxpoints);
				Imgproc.drawContours(m_srcImage, matofapprox, 0, new Scalar(20, 25, 200), 5);

				//If its a tetragon, (ex, rectangle, trapezoid), add the convex hulls to the list
				if(approx.rows()==4) {
					targetHulls.add(approxpoints);
					rRects.add(rRect);
				}
				if (tempArea > biggestArea)
				{
					biggestRect = rRect;
					biggestArea = tempArea;
				}
				Point[] vertices = new Point[4];
				rRect.points(vertices);
				double Center_Y = rRect.center.y;
				double Center_X = rRect.center.x;

				Scalar color = new Scalar(255, 0, 0);

				Imgproc.line(m_srcImage, new Point(Center_X, Center_Y - 50), new Point(Center_X, Center_Y + 50), color, 2);
				//System.out.println(contourSolidity);
			}
		}

		ArrayList<Tuple<Target, Target>> ListOfPairs = new ArrayList<>();
		//We need to make pairs of targets for further calculation
		//First we should sort the targetHulls by position along x
		ArrayList<Target> ListOfTargets = new ArrayList<>();
		for(int i = 0; i < targetHulls.size(); i++){
			ListOfTargets.add(new Target(rRects.get(i), targetHulls.get(i)));
		}

		Collections.sort(ListOfTargets, new Comparator<Target>() {
			@Override
			public int compare(Target o1, Target o2) {
				return o1.rect.center.x < o2.rect.center.x ? -1 : o1.rect.center.x == o2.rect.center.x ? 0:1;
			}
		});

		Boolean skip = false;
		for(int i = 0; i < ListOfTargets.size(); i++){
			double angle = Math.abs(ListOfTargets.get(i).rect.angle%90);
			if(skip){
				skip = false;
				continue;
			}
			if(angle > 60 && angle < 90 && i+1<targetHulls.size()) {
				ListOfPairs.add(new Tuple(ListOfTargets.get(i), ListOfTargets.get(i+1)));
				skip = true;
				continue;
			}
		}
		double highestDistance = 0;
		int chosenIndex = 0;
		for(int i = 0; i<ListOfPairs.size(); i++){
			Tuple<Target, Target> tempTarget = ListOfPairs.get(i);
			//Calculate mean point:
			Point mean = new Point((tempTarget.x.rect.center.x + tempTarget.y.rect.center.x)/2, (tempTarget.x.rect.center.y + tempTarget.y.rect.center.y)/2);
			//Calcualte distance
			double distance = Math.sqrt(Math.pow((tempTarget.x.rect.center.x - tempTarget.y.rect.center.x),2) + Math.pow(tempTarget.x.rect.center.y = tempTarget.y.rect.center.y, 2));
			//calculate angle of mean point
			double degree = Math.atan(((mean.x-CENTRE_IMAGE_X)*CONV_PIXEL2CM)/FOV_LENGHT)*2;
			degree = Math.toDegrees(degree);
			if(distance > highestDistance) {
				chosenIndex = i;
				m_degree = degree;
			}
			MatOfPoint combinedPoints = new MatOfPoint();
			ArrayList<Mat> concatList = new ArrayList<>();
			concatList.add(tempTarget.x.hull);
			concatList.add(tempTarget.y.hull);
			Core.vconcat(concatList, combinedPoints);
			ArrayList<MatOfPoint> pointsList = new ArrayList<>();
			pointsList.add(combinedPoints);
			Rect boundingRect = Imgproc.boundingRect(combinedPoints);
			Point[] vertices = new Point[4];
			RotatedRect rRect = new RotatedRect();
			rRect.angle = 0;
			rRect.size = boundingRect.size();
			rRect.center = new Point(boundingRect.x + (boundingRect.size().width/2), boundingRect.y + (boundingRect.size().height/2));
			rRect.points(vertices);
			for (int j = 0; j < 4; j++) //Dessiner un rectangle avec rotation
			{
				Imgproc.line(m_srcImage, vertices[j], vertices[(j+1)%4], new Scalar(20,200,40), 5);
			}
			Imgproc.drawMarker(m_srcImage, mean, new Scalar(40, 200, 200), Imgproc.MARKER_CROSS, 40, 1, Imgproc.LINE_AA);
		}

		Core.add(m_srcImage, backupImage, m_srcImage);
	}

	public void highGoal() {
		m_log.info("[runTarget] highGoal started");
		//this is a test code
		m_degree = 30;
	}

	public void gearTerre() {
		m_log.info("[runTarget] gearTerre started");
		//this is a test code
		m_degree = 40;
	}

	public void balleTerre() {
		m_log.info("[runTarget] balleTerre started");
		//this is a test code
		m_degree = 50;
	}


	///		[Category] Logging
	public void createLog() { //utiliser lors de l'initialisation
		m_log = Logger.getLogger("rbtxVisionLog");
		try {
			// This block configure the logger with handler and formatter
			//fh = new FileHandler("$HOME/JavaCodeTest/Log.log");
			m_fh = new FileHandler("./vision.log");
			m_log.addHandler(m_fh);
			SimpleFormatter formatter = new SimpleFormatter();
			m_fh.setFormatter(formatter);
			m_log.setLevel(Level.INFO);
		}
		catch (IOException e) 
        {
			e.printStackTrace();
		}
        m_log.info("Log created successfully");
        
	}
}
	



