package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * A class to identify and make available April Tags using the Vision Portal.
 */
public class FtcAprilTag {
  static final String TAG = "FtcAprilTag";
  public static final String APRIL_TAG_SERVO_NAME = "aprilTagServo";
  public static final double OBELISK_BLUE_POSITION = 0.5335;
  public static final double OBELISK_RED_POSITION = 0.4705;
  public static final double GOAL_POSITION = 0.50;
  private AprilTagProcessor aprilTagProcessor;
  private VisionPortal visionPortal;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry;
  private final FtcBot parent;
  private FtcServo aprilTagServo = null;

  public FtcAprilTag(FtcBot robot) {
    parent = robot;
  }

  private List<AprilTagDetection> getAllDetections() {
    List<AprilTagDetection> detections = null;
    if (aprilTagProcessor != null) {
      detections = aprilTagProcessor.getDetections();
    }

    return detections;
  }

  public double getGoalRange() {
    double range = Double.MIN_VALUE;
    List<AprilTagDetection> detections = getAllDetections();
    if (detections != null && !detections.isEmpty() && parent != null && parent.config != null) {
      for (AprilTagDetection detection : detections) {
        if (detection.metadata != null) {
          if (parent.config.allianceColor == AllianceColorEnum.BLUE && detection.id == 20) {
            range = detection.ftcPose.range;
          } else if (parent.config.allianceColor == AllianceColorEnum.RED && detection.id == 24) {
            range = detection.ftcPose.range;
          }
        }
      }
    }

    return range;
  }

  /**
   * Initialize the object detection processor.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;

    aprilTagServo = new FtcServo(hardwareMap.get(Servo.class, APRIL_TAG_SERVO_NAME));
    aprilTagServo.setPosition(FtcServo.MID_POSITION);

    // Create the AprilTag processor.
    aprilTagProcessor = new AprilTagProcessor.Builder()
        .setDrawAxes(FtcUtils.DEBUG)
        .setDrawCubeProjection(false)
        .setDrawTagOutline(FtcUtils.DEBUG)
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        .build();

    visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, FtcOpenCvCam.WEBCAM_2_NAME))
        .setCameraResolution(new Size(FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT))
        .enableLiveView(true)
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .setAutoStopLiveView(true)
        .addProcessor(aprilTagProcessor)
        .build();

    aprilTagServo = new FtcServo(hardwareMap.get(Servo.class, APRIL_TAG_SERVO_NAME));
    aprilTagServo.setDirection(Servo.Direction.FORWARD);

    if (FtcUtils.DEBUG) {
      FtcDashboard dashboard = FtcDashboard.getInstance();
      if (dashboard != null) {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 0);
      }

      telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
      telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    FtcLogger.exit();
  }

  public void pointAtObelisk() {
    if (parent != null && parent.config != null) {
      if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
        aprilTagServo.setPosition(OBELISK_BLUE_POSITION);
      } else if (parent.config.allianceColor == AllianceColorEnum.RED) {
        aprilTagServo.setPosition(OBELISK_RED_POSITION);
      }
    }
  }

  /**
   * Function to add telemetry about AprilTag detections.
   */
  @SuppressLint("DefaultLocale")
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled && aprilTagProcessor != null) {
      List<AprilTagDetection> aprilTagDetections = getAllDetections();
      telemetry.addData("# AprilTags Detected", aprilTagDetections.size());

      // Step through the list of detections and set info for each one.
      for (AprilTagDetection detection : aprilTagDetections) {
        if (detection.metadata != null) {
          telemetry.addLine(String.format("\n==== (ID %d) %s",
              detection.id, detection.metadata.name));
          telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)",
              detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
          telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)",
              detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
          telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
              detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
          telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
        }

        telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)",
            detection.center.x, detection.center.y));
      }

      // Add "key" information to telemetry
      telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
      telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
      telemetry.addLine("RBE = Range, Bearing & Elevation");
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

    if (aprilTagServo != null) {
      aprilTagServo.setPosition(GOAL_POSITION);
    }
    FtcLogger.exit();
  }

  /**
   * Stops the tag detection processing.
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
