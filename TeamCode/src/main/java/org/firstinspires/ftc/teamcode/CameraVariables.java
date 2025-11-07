package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cameraProcessor.BlobProcessor;
import org.firstinspires.ftc.teamcode.cameraProcessor.EdgeProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

@Config

public class CameraVariables {
    public static int minGain;
    public static int maxGain;
    protected static Bitmap lastBitmap;
    public static int gain = 20;
    public static int exposure = 50;
    protected static int minExposure;
    protected static int maxExposure;

    // TestProcessor
    public static Mat cameraStream1;
    public static Mat cameraStream2;
    // BlobFinder
    protected static  VisionPortal visionPortal; // Used to manage the video source.
    protected static  VisionPortal visionPortal2; // Used to manage the video source.

    protected static  AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    protected static  AprilTagProcessor aprilTag2;
    protected static BlobProcessor blob;
    protected static EdgeProcessor edge;
    protected static  WebcamName webcam1, webcam2;

    // ManeVision
    public static HardwareMap hardwareMap;
    public static Size cameraResolution = new Size(640, 480);

    // Canny edge finder
    public static double threshold1 = 100; // Lower threshold
    public static double threshold2 = 200; // Upper threshold
    public static int apertureSize = 3; // Size of the Sobel operator
    public static boolean l2Gradient = false; // More accurate gradient calculation
    public static boolean findEdges = false;

    // BlobProcessor
    public static int blurLevel = 10;
    public static int lowerScalar1 = 0;
    public static int lowerScalar2 = 150;
    public static int lowerScalar3 = 0;
    public static int upperScalar1 = 150;
    public static int upperScalar2 = 255;
    public static int upperScalar3 = 255;
    public static int erodeLevel = 15;
    public static int dilateLevel = 15;
    public static double distanceX1 = 0;
    public static double distanceY1 = 0;
    public static double distanceX2 = 0;
    public static double distanceY2 = 0;
    public static boolean drawBox = false;

    public static double angleMultiplier = 0.001;
    public static double driveMultiplier = 0.001;
    public static double maxPow = .3;



    public  static String matType = "";

    public static double camHeight = 7.7;
    public static double camOffset = 90-69.1;
    public static double cameraAngleFromRobotFront = 135;
    public static double cameraXFromRobotCenter = -2.5;
    public static double cameraYFromRobotCenter = 4.5;
    public static String cameraLEDDirection = "top"; // left, top, right, bottom

    public static Mat rawCameraStream = null;

    public static Mat matForDisplay = null;
}