package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class ArtifactSensor {
  public static final String SORTER_SERVO_NAME = "sorterServo";
  private PredominantColorProcessor colorProcessor;
  private VisionPortal visionPortal;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry;

  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;
    colorProcessor = new PredominantColorProcessor.Builder()
        .setRoi(ImageRegion.entireFrame())
        .setSwatches(
            PredominantColorProcessor.Swatch.RED,
            PredominantColorProcessor.Swatch.ORANGE,
            PredominantColorProcessor.Swatch.YELLOW,
            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
            PredominantColorProcessor.Swatch.BLUE,
            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
            PredominantColorProcessor.Swatch.BLACK,
            PredominantColorProcessor.Swatch.WHITE)
        .build();

    visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, FtcOpenCvCam.WEBCAM_1_NAME))
        .setCameraResolution(new Size(FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT))
        .enableLiveView(true)
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .setAutoStopLiveView(true)
        .addProcessor(colorProcessor)
        .build();

    if (FtcUtils.DEBUG) {
      FtcDashboard dashboard = FtcDashboard.getInstance();
      if (dashboard != null) {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 0);
      }

      telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
      telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }
  }

  public PredominantColorProcessor.Swatch getSwatch() {
    PredominantColorProcessor.Swatch swatch = PredominantColorProcessor.Swatch.BLACK;
    if (colorProcessor != null) {
      swatch = colorProcessor.getAnalysis().closestSwatch;
    }

    return swatch;
  }

  /**
   * Function to add telemetry about AprilTag detections.
   */
  @SuppressLint("DefaultLocale")
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled) {
      telemetry.addData("Artifact color", getSwatch().name());
    }

    FtcLogger.exit();
  }

  /**
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (visionPortal != null) {
      visionPortal.resumeStreaming();
    }

    FtcLogger.exit();
  }

  /**
   * Stops the object detection processing.
   */
  public void stop() {
    FtcLogger.enter();
    if (visionPortal != null) {
      visionPortal.close();
      visionPortal = null;
    }

    FtcLogger.exit();
  }
}
