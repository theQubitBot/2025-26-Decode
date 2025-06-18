package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

public class ObjectDetectionByChannel extends ObjectDetectionBase {

    private final Mat yCrCbMat;
    private final Mat Cr;
    private final Mat Cb;
    private Mat regionCenterCr, regionRightCr;
    private Mat regionCenterCb, regionRightCb;
    private int avgCenter, avgRight;
    private final int avgThreshold = 130;
    private RotatedRect rRect;

    public ObjectDetectionByChannel() {
        yCrCbMat = new Mat();
        Cr = new Mat();
        Cb = new Mat();
        regionCenterCr = new Mat();
        regionCenterCb = new Mat();
        regionRightCr = new Mat();
        regionRightCb = new Mat();
    }

    /**
     * PERFORMANCE
     * Initializes various Mats and sub mats so that we don't need to
     * reinitialize them on every frame..
     *
     * @param firstFrame The very first frame that is processed.
     */
    public void init(Mat firstFrame) {
        Imgproc.cvtColor(firstFrame, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCrCbMat, Cb, FtcColorUtils.CB_CHANNEL);
        Core.extractChannel(yCrCbMat, Cr, FtcColorUtils.CR_CHANNEL);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any change to the child affects the parent and vice-versa.
         */
        regionCenterCb = Cb.submat(centerRect);
        regionRightCb = Cb.submat(rightRect);

        regionCenterCr = Cr.submat(centerRect);
        regionRightCr = Cr.submat(rightRect);
    }

    /**
     * Process the current frame, looking for the game element.
     *
     * @param frame         The input frame to process.
     * @param gameElement   The game element to look for.
     * @param annotateFrame When true,draws annotations on the frame. Helps with debugging.
     */
    @SuppressLint("DefaultLocale")
    public void processFrame(Mat frame, GameElement gameElement, boolean annotateFrame) {
        Imgproc.cvtColor(frame, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCrCbMat, Cb, FtcColorUtils.CB_CHANNEL);
        Core.extractChannel(yCrCbMat, Cr, FtcColorUtils.CR_CHANNEL);

        // Process Image
        if (gameElement.tag.equals(FtcColorUtils.TAG_BLUE)) {
            avgCenter = (int) Core.mean(regionCenterCb).val[0];
            avgRight = (int) Core.mean(regionRightCb).val[0];
        } else if (gameElement.tag.equals(FtcColorUtils.TAG_RED1) || gameElement.tag.equals(FtcColorUtils.TAG_RED2)) {
            avgCenter = (int) Core.mean(regionCenterCr).val[0];
            avgRight = (int) Core.mean(regionRightCr).val[0];
        } else {
            avgCenter = avgRight = 0;
        }

        gameElement.invalidate();
        if (avgCenter > avgThreshold) {
            rRect = new RotatedRect(FtcUtils.getMidpoint(centerRect), centerRect.size(), 0);
            gameElement.validate(rRect, FtcUtils.getAspectRatio(centerRect), centerRect.area());
            if (FtcUtils.DEBUG || annotateFrame) {
                Imgproc.rectangle(frame, gameElement.boundingRect, FtcColorUtils.RGB_YELLOW, gameElement.borderSize);
                Imgproc.putText(frame, String.format("%s: Avg %d", gameElement.tag, avgCenter),
                        new Point(gameElement.boundingRect.x, gameElement.boundingRect.y - 40),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, gameElement.elementColor, gameElement.borderSize);
            }
        } else if (avgRight > avgThreshold) {
            rRect = new RotatedRect(FtcUtils.getMidpoint(rightRect), rightRect.size(), 0);
            gameElement.validate(rRect, FtcUtils.getAspectRatio(rightRect), rightRect.area());
            if (FtcUtils.DEBUG || annotateFrame) {
                Imgproc.rectangle(frame, gameElement.boundingRect, FtcColorUtils.RGB_YELLOW, gameElement.borderSize);
                Imgproc.putText(frame, String.format("%s: Avg %d", gameElement.tag, avgRight),
                        new Point(gameElement.boundingRect.x, gameElement.boundingRect.y - 40),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, gameElement.elementColor, gameElement.borderSize);
            }
        }
    }
}
