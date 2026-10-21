package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Bitmap.Config;
import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

@TeleOp(
        name = "Test Camera Stream",
        group = "Dev"
)
@Disabled
public class TestCameraStream extends OpMode {
    @NotNull
    private Processor processor = new Processor();

    @Override
    public void init() {
        new VisionPortal.Builder().addProcessor(this.processor).setCamera(BuiltinCameraDirection.BACK).build();
        PanelsCameraStream.INSTANCE.startStream(this.processor, null);
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        PanelsCameraStream.INSTANCE.stopStream();
    }

    public static class Processor implements VisionProcessor, CameraStreamSource {
        @NotNull
        private final AtomicReference<Bitmap> lastFrame;

        public Processor() {
            byte width = 1;
            byte height = 1;
            Bitmap.Config config = Config.RGB_565;
            this.lastFrame = new AtomicReference<>(Bitmap.createBitmap(width, height, config));
        }

        @Override
        public void init(int width, int height, @Nullable CameraCalibration calibration) {
            Bitmap.Config config = Config.RGB_565;
            this.lastFrame.set(Bitmap.createBitmap(width, height, config));
        }

        @Nullable
        @Override
        public Object processFrame(@NotNull Mat frame, long captureTimeNanos) {
            int width = frame.width();
            int height = frame.height();
            Bitmap.Config config = Config.RGB_565;
            Bitmap b = Bitmap.createBitmap(width, height, config);
            Utils.matToBitmap(frame, b);
            this.lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(@NotNull Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, @Nullable Object userContext) {
        }

        @Override
        public void getFrameBitmap(@NotNull Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(consumer -> consumer.accept(lastFrame.get()));
        }
    }
}
