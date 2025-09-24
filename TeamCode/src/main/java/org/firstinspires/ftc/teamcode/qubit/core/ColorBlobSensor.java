package org.firstinspires.ftc.teamcode.qubit.core;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ColorBlobSensor {

    private VisionPortal visionPortal;
    private final ColorProcessor colorProcessor;

    public ColorBlobSensor() {
        colorProcessor = new ColorProcessor();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorProcessor)
                .build();
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }


    public double getPurpleArea() {
        return colorProcessor.getPurpleArea();
    }

    public double getGreenArea() {
        return colorProcessor.getGreenArea();
    }

    public String getDominantColor() {
        return colorProcessor.getDominantColor();
    }


    private static class ColorProcessor implements VisionProcessor { // uses HSV color bounds but might have to change

        private final Scalar lowerPurple = new Scalar(125, 50, 50); // HSV lower purple bound
        private final Scalar upperPurple = new Scalar(155, 255, 255); // HSV upper purple bound

        private final Scalar lowerGreen = new Scalar(40, 50, 50); // HSV lower green bound
        private final Scalar upperGreen = new Scalar(80, 255, 255); // HSV upper green bound

        private double purpleArea = 0;
        private double greenArea = 0;


        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // nothing to init?
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat purpleMask = new Mat();
            Mat greenMask = new Mat();

            Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);
            Core.inRange(hsv, lowerGreen, upperGreen, greenMask);

            purpleArea = Core.countNonZero(purpleMask);
            greenArea = Core.countNonZero(greenMask);

            // Draw quick debug overlays
            if (purpleArea > 500) {
                Imgproc.rectangle(input, new Rect(new Point(20, 20), new Point(120, 60)),
                        new Scalar(255, 0, 255), 2);
                Imgproc.putText(input, "Purple", new Point(25, 50),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 255), 2);
            }
            if (greenArea > 500) {
                Imgproc.rectangle(input, new Rect(new Point(140, 20), new Point(240, 60)),
                        new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, "Green", new Point(145, 50),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
            }

            hsv.release();
            purpleMask.release();
            greenMask.release();

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }


        public double getPurpleArea() {
            return purpleArea;
        }

        public double getGreenArea() {
            return greenArea;
        }

        public String getDominantColor() {
            if (purpleArea > greenArea && purpleArea > 1000) {
                return "Purple";
            } else if (greenArea > purpleArea && greenArea > 1000) {
                return "Green";
            }
            return "None";
        }
    }
}
