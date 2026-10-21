package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class AprilTagStreamProcessor implements VisionProcessor, CameraStreamSource {
    private final AprilTagProcessor aprilTagProcessor;
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public AprilTagStreamProcessor(AprilTagProcessor aprilTagProcessor) {
        this.aprilTagProcessor = aprilTagProcessor;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        aprilTagProcessor.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Run AprilTag processing
        aprilTagProcessor.processFrame(frame, captureTimeNanos);

        // Convert the frame to a Bitmap for streaming
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return getDetections();
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // This method is optional, but you could draw detections here if needed
        aprilTagProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }
}

