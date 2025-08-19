package org.firstinspires.ftc.teamcode.cameraProcessor;

import org.firstinspires.ftc.teamcode.CameraVariables;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class EdgeProcessor {
    Mat mask;
    Mat displayed;
    public Mat processFrame(Mat frame, int id) {
//        if (CameraVariables.findEdges) {
//            Mat newMat1 = new Mat();
//            mat1.convertTo(newMat1, CvType.CV_8U);
//            Mat newMat1B = new Mat();
//            Imgproc.Canny(newMat1, newMat1B, CameraVariables.threshold1, CameraVariables.threshold2);
//            newMat1B.convertTo(mat1, CvType.CV_8UC4);
////            mat1 = newMat1B;
//        }
        displayed = findEdges(frame);
        return displayed;
    }
    private Mat findEdges(Mat inputMat) {
        Mat outputMat = new Mat();
        Imgproc.Canny(inputMat, outputMat, CameraVariables.threshold1, CameraVariables.threshold2);
        return outputMat;
    }

}
