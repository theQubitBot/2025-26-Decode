package org.firstinspires.ftc.teamcode.qubit.core;

import org.firstinspires.ftc.teamcode.qubit.core.enumerations.PipelineStageEnum;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StageRenderPipeline extends OpenCvPipeline {
    private static final String TAG = "StageRenderPipeline";
    Mat yCbCrMat = new Mat();
    Mat yCrMat = new Mat();
    Mat yCbMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    final Object contourSyncObject = new Object();

    PipelineStageEnum stageToRenderToViewport = PipelineStageEnum.YCbCr_CHANNEL1;
    final PipelineStageEnum[] stages = PipelineStageEnum.values();

    Point textAnchor = new Point(20, 20);

    public void decrementStageToRenderToViewport() {
        synchronized (stages) {
            stageToRenderToViewport =
                    stages[(stageToRenderToViewport.ordinal() - 1 + stages.length) % stages.length];
        }
    }

    public void incrementStageToRenderToViewport() {
        synchronized (stages) {
            stageToRenderToViewport = stages[(stageToRenderToViewport.ordinal() + 1) % stages.length];
        }
    }

    public int getContourCount() {
        int count;
        synchronized (contourSyncObject) {
            count = contoursList.size();
        }

        return count;
    }

    public PipelineStageEnum getStageToRenderToViewport() {
        PipelineStageEnum stageEnum;
        synchronized (stages) {
            stageEnum = stageToRenderToViewport;
        }

        return stageEnum;
    }

    @Override
    public Mat processFrame(Mat input) {
        synchronized (contourSyncObject) {
            contoursList.clear();
        }

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */
        Imgproc.cvtColor(input, yCbCrMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrMat, yCrMat, 1);
        Core.extractChannel(yCbCrMat, yCbMat, 2);
        Imgproc.threshold(yCbMat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
        synchronized (contourSyncObject) {
            Imgproc.findContours(thresholdMat, contoursList, new Mat(),
                    Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        }

        input.copyTo(contoursOnFrameMat);
        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1,
                new Scalar(0, 0, 255), 3, 8);

        Imgproc.putText(input, "Ftc Test", textAnchor,
                Imgproc.FONT_HERSHEY_SIMPLEX, 1.5, FtcColorUtils.RGB_GREEN, 2);
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols() / 4.0,
                        input.rows() / 4.0),
                new Point(
                        input.cols() * (3f / 4f),
                        input.rows() * (3f / 4f)),
                FtcColorUtils.RGB_GREEN, 4);


        switch (getStageToRenderToViewport()) {
            case YCbCr_CHANNEL1: {
                return yCrMat;
            }

            case YCbCr_CHANNEL2: {
                return yCbMat;
            }

            case THRESHOLD: {
                return thresholdMat;
            }

            case CONTOURS_OVERLAID_ON_FRAME: {
                return contoursOnFrameMat;
            }

            case RAW_IMAGE:
            default: {
                return input;
            }
        }
    }

    @Override
    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */
        incrementStageToRenderToViewport();
    }
}
