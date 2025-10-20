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
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;
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
  public static final double OBELISK_BLUE_ALLIANCE_LARGE_TRIANGLE_POSITION = 0.5365;
  public static final double OBELISK_BLUE_ALLIANCE_SMALL_TRIANGLE_POSITION = 0.5050;
  public static final double OBELISK_RED_ALLIANCE_LARGE_TRIANGLE_POSITION = 0.4465;
  public static final double OBELISK_RED_ALLIANCE_SMALL_TRIANGLE_POSITION = 0.4780;
  public static final double GOAL_POSITION = 0.4915;
  public static final double MAX_RANGE = 150.0;
  public static final double MIN_RANGE = 0.0;
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
    double range = 0.0;
    List<AprilTagDetection> detections = getAllDetections();
    if (detections != null && !detections.isEmpty() && parent != null && parent.config != null) {
      for (AprilTagDetection detection : detections) {
        if (detection.metadata != null) {
          if ((parent.config.allianceColor == AllianceColorEnum.BLUE && detection.id == 20) ||
              (parent.config.allianceColor == AllianceColorEnum.RED && detection.id == 24)) {
            range = detection.ftcPose.range;
          }
        }
      }
    }

    range = com.qualcomm.robotcore.util.Range.clip(range, MIN_RANGE, MAX_RANGE);
    return range;
  }

  /**
   * Initialize the AprilTag detection processor.
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
        .build();

    visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, FtcOpenCvCam.WEBCAM_2_NAME))
        .setCameraResolution(new Size(FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT))
        .enableLiveView(FtcUtils.DEBUG)
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .setAutoStopLiveView(true)
        .addProcessor(aprilTagProcessor)
        .build();

    aprilTagServo = new FtcServo(hardwareMap.get(Servo.class, APRIL_TAG_SERVO_NAME));
    aprilTagServo.setDirection(Servo.Direction.FORWARD);

    if (FtcUtils.DEBUG) {
      visionPortal.resumeLiveView();
      FtcDashboard dashboard = FtcDashboard.getInstance();
      if (dashboard != null) {
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 0);
      }
    } else {
      visionPortal.stopLiveView();
    }

    FtcLogger.exit();
  }

  public void pointAtObelisk() {
    if (parent != null && parent.config != null) {
      if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
        if (parent.config.robotPosition == RobotPositionEnum.LARGE_TRIANGLE) {
          aprilTagServo.setPosition(OBELISK_BLUE_ALLIANCE_LARGE_TRIANGLE_POSITION);
        } else {
          aprilTagServo.setPosition(OBELISK_BLUE_ALLIANCE_SMALL_TRIANGLE_POSITION);
        }
      } else if (parent.config.allianceColor == AllianceColorEnum.RED) {
        if (parent.config.robotPosition == RobotPositionEnum.LARGE_TRIANGLE) {
          aprilTagServo.setPosition(OBELISK_RED_ALLIANCE_LARGE_TRIANGLE_POSITION);
        } else {
          aprilTagServo.setPosition(OBELISK_RED_ALLIANCE_SMALL_TRIANGLE_POSITION);
        }
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
      double position = aprilTagServo.getPosition();
      telemetry.addData(TAG, "Servo %5.4f (%s)",
          position,
          position == OBELISK_BLUE_ALLIANCE_LARGE_TRIANGLE_POSITION ? "Blue large" :
              position == OBELISK_BLUE_ALLIANCE_SMALL_TRIANGLE_POSITION ? "Blue small" :
                  position == OBELISK_RED_ALLIANCE_LARGE_TRIANGLE_POSITION ? "Red large" :
                      position == OBELISK_RED_ALLIANCE_SMALL_TRIANGLE_POSITION ? "Red small" :
                          position == GOAL_POSITION ? "Goal" : "Unknown");
      List<AprilTagDetection> aprilTagDetections = getAllDetections();
      telemetry.addData("AprilTags count", aprilTagDetections.size());

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
   * Stops the April Tag detection processing.
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
