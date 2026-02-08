package org.firstinspires.ftc.teamcode.cameraProcessor;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.List;

/**
 * Throttled wrapper around AprilTagProcessor to limit processing rate and reduce CPU load.
 */
public class ThrottledAprilTagProcessor implements VisionProcessor {

    private final AprilTagProcessor delegate;
    private final double minIntervalMs;
    private long lastProcessTimeMs = 0;
    private int skippedFrames = 0;
    private int processedFrames = 0;

    private long lastProcessedTimeMs = 0;
    private double approximateFps = 0.0;

    private Telemetry telemetry = null;

    public static class Builder {
        private final AprilTagProcessor.Builder delegateBuilder = new AprilTagProcessor.Builder();
        private double maxProcessRateHz = 10.0;

        public Builder setMaxProcessRateHz(double hz) {
            this.maxProcessRateHz = Math.max(0.5, Math.min(30.0, hz));
            return this;
        }

        public Builder setTagFamily(AprilTagProcessor.TagFamily family) {
            delegateBuilder.setTagFamily(family);
            return this;
        }

        public Builder setDrawTagID(boolean draw) {
            delegateBuilder.setDrawTagID(draw);
            return this;
        }

        public Builder setDrawTagOutline(boolean draw) {
            delegateBuilder.setDrawTagOutline(draw);
            return this;
        }

        public Builder setDrawAxes(boolean draw) {
            delegateBuilder.setDrawAxes(draw);
            return this;
        }

        public Builder setDrawCubeProjection(boolean draw) {
            delegateBuilder.setDrawCubeProjection(draw);
            return this;
        }

        public Builder setLensIntrinsics(double fx, double fy, double cx, double cy) {
            delegateBuilder.setLensIntrinsics(fx, fy, cx, cy);
            return this;
        }

        public Builder setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit) {
            delegateBuilder.setOutputUnits(distanceUnit, angleUnit);
            return this;
        }

        public ThrottledAprilTagProcessor build() {
            return new ThrottledAprilTagProcessor(delegateBuilder.build(), maxProcessRateHz);
        }
    }

    private ThrottledAprilTagProcessor(AprilTagProcessor delegate, double maxRateHz) {
        this.delegate = delegate;
        this.minIntervalMs = 1000.0 / maxRateHz;
    }

    @Override
    public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
        delegate.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        long nowMs = System.currentTimeMillis();

        if (nowMs - lastProcessTimeMs >= minIntervalMs) {
            // Call delegate — it modifies/annotates 'input' in-place if drawing is enabled
            delegate.processFrame(input, captureTimeNanos);

            if (lastProcessedTimeMs > 0) {  // avoid div-by-zero first time
                double deltaSec = (nowMs - lastProcessedTimeMs) / 1000.0;
                approximateFps = 1.0 / deltaSec;  // instantaneous FPS
            }

            lastProcessTimeMs = nowMs;
            processedFrames++;

            if (telemetry != null) {
                telemetry.addData("Throttled AprilTag FPS", "%.1f (processed: %d, skipped: %d)",
                        1000.0 / minIntervalMs, processedFrames, skippedFrames);
            }

            return null;  // No custom context needed — matches AprilTagProcessor
        } else {
            skippedFrames++;
            return null;  // Or return input; but null is consistent
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Forward to delegate so built-in drawing (tags, axes, cubes) still works
        delegate.onDrawFrame(canvas, onscreenWidth, onscreenHeight,
                scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    // Public API passthroughs
    public List<AprilTagDetection> getDetections() {
        return delegate.getDetections();
    }

    public List<AprilTagDetection> getFreshDetections() {
        return delegate.getFreshDetections();
    }

    //    public double getFps() {
//        return delegate.getFps();
//    }
    public double getApproximateFps() {
        return approximateFps;
    }



    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public AprilTagProcessor getDelegate() {
        return delegate;
    }
}
