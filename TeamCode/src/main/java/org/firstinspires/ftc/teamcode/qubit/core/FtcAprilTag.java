package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * A class to identify and make available April Tags using the Vision Portal.
 */
public class FtcAprilTag extends FtcSubSystemBase {
  public static final String TAG = "FtcAprilTag";
  public static final String WEBCAM_1_NAME = "Webcam 1";
  public static final String WEBCAM_2_NAME = "Webcam 2";

  public static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
  public static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
  public static final String APRIL_TAG_SERVO_NAME = "aprilTagServo";
  private static final double OBELISK_BLUE_ALLIANCE_GOAL_POSITION = 0.5380;
  private static final double OBELISK_BLUE_ALLIANCE_AUDIENCE_POSITION = 0.5000;
  private static final double OBELISK_RED_ALLIANCE_GOAL_POSITION = 0.4465;
  private static final double OBELISK_RED_ALLIANCE_AUDIENCE_POSITION = 0.4870;
  private static final double GOAL_POSITION = 0.4917;
  private static final double MAX_RANGE = 140.0;
  private static final double MIN_RANGE = 0.0;
  private static final int CAMERA_EXPOSURE = 10; // milliseconds
  private static final int CAMERA_GAIN = 40;
  // Vision/Sensor limits
  private static final double APRILTAG_BEARING_MIN = -20.0;
  private static final double APRILTAG_BEARING_MAX = 20.0;
  private AprilTagProcessor aprilTagProcessor;
  private VisionPortal visionPortal;
  private FtcAprilTagAsyncUpdater asyncUpdater = null;
  private Thread asyncUpdaterThread = null;

  public int currentExposure, minExposure, maxExposure;
  public int currentGain, minGain, maxGain;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry;
  private final BaseBot parent;
  private FtcServo aprilTagServo = null;

  public FtcAprilTag(BaseBot robot) {
    parent = robot;
  }

  private List<AprilTagDetection> getAllDetections() {
    List<AprilTagDetection> detections = null;
    if (asyncUpdater != null) {
      detections = asyncUpdater.getAllDetections();
    }

    return detections;
  }

  /**
   * Gets and stores exposure and gain ranges.
   */
  private void getCameraSetting() {
    if (visionPortal != null) {
      // Wait for the camera to be open
      if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
        // Get camera control values unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        currentExposure = (int) exposureControl.getExposure(TimeUnit.MILLISECONDS);
        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        currentGain = gainControl.getGain();
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();
      }
    }
  }

  private AprilTagPoseFtc getFtcPose() {
    AprilTagPoseFtc pose = null;
    List<AprilTagDetection> detections = getAllDetections();
    if (detections != null && !detections.isEmpty() && parent != null && parent.config != null) {
      for (AprilTagDetection detection : detections) {
        if (detection.metadata != null) {
          if ((parent.config.allianceColor == AllianceColorEnum.BLUE && detection.id == 20) ||
              (parent.config.allianceColor == AllianceColorEnum.RED && detection.id == 24)) {
            pose = detection.ftcPose;
            break;
          }
        }
      }
    }

    return pose;
  }

  public double getGoalDistance() {
    double distance = 0;
    AprilTagPoseFtc pose = getFtcPose();
    if (pose != null) {
      distance = com.qualcomm.robotcore.util.Range.clip(pose.range, MIN_RANGE, MAX_RANGE);
    }

    return distance;
  }

  public double getHeading() {
    double heading = 0;
    AprilTagPoseFtc pose = getFtcPose();
    if (pose != null) {
      // camera is rotated, so we use pitch for heading
      heading = com.qualcomm.robotcore.util.Range.clip(pose.pitch, APRILTAG_BEARING_MIN, APRILTAG_BEARING_MAX);
    }

    return heading;
  }

  public ObeliskTagEnum getObeliskTag() {
    ObeliskTagEnum ote = ObeliskTagEnum.UNKNOWN;
    List<AprilTagDetection> detections = getAllDetections();
    if (detections != null && !detections.isEmpty()) {
      for (AprilTagDetection detection : detections) {
        if (detection.metadata != null) {
          if (detection.id == 21) {
            ote = ObeliskTagEnum.GPP;
            break;
          } else if (detection.id == 22) {
            ote = ObeliskTagEnum.PGP;
            break;
          } else if (detection.id == 23) {
            ote = ObeliskTagEnum.PPG;
            break;
          }
        }
      }
    }

    return ote;
  }

  /**
   * Initialize the AprilTag detection processor.
   */
  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;

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
        .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_2_NAME))
        .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
        .setLiveViewContainerId(0)
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .setAutoStopLiveView(true)
        .addProcessor(aprilTagProcessor)
        .build();

    Deadline deadline = new Deadline(5, TimeUnit.SECONDS);
    while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING &&
        !deadline.hasExpired()) {
      FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    updateCameraSettings(CAMERA_EXPOSURE, CAMERA_GAIN);
    getCameraSetting();
    aprilTagServo = new FtcServo(hardwareMap.get(Servo.class, APRIL_TAG_SERVO_NAME));
    aprilTagServo.setDirection(Servo.Direction.FORWARD);

    if (FtcUtils.DEBUG || parent == null) {
      visionPortal.resumeLiveView();
      FtcDashboard dashboard = FtcDashboard.getInstance();
      if (dashboard != null) {
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 0);
      }
    }

    stopAsyncOperations();
    startAsyncOperations();

    FtcLogger.exit();
  }

  public void pointAtObelisk() {
    if (parent != null && parent.config != null) {
      if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
        if (parent.config.robotPosition == RobotPositionEnum.GOAL) {
          aprilTagServo.setPosition(OBELISK_BLUE_ALLIANCE_GOAL_POSITION);
        } else {
          aprilTagServo.setPosition(OBELISK_BLUE_ALLIANCE_AUDIENCE_POSITION);
        }
      } else if (parent.config.allianceColor == AllianceColorEnum.RED) {
        if (parent.config.robotPosition == RobotPositionEnum.GOAL) {
          aprilTagServo.setPosition(OBELISK_RED_ALLIANCE_GOAL_POSITION);
        } else {
          aprilTagServo.setPosition(OBELISK_RED_ALLIANCE_AUDIENCE_POSITION);
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
    if (telemetryEnabled && telemetry != null && aprilTagProcessor != null) {
      double position = aprilTagServo.getPosition();
      telemetry.addData(TAG, "Servo %5.4f (%s)",
          position,
          FtcUtils.areEqual(position, OBELISK_BLUE_ALLIANCE_GOAL_POSITION, FtcUtils.EPSILON4) ? "Blue goal obelisk" :
              FtcUtils.areEqual(position, OBELISK_BLUE_ALLIANCE_AUDIENCE_POSITION, FtcUtils.EPSILON4) ? "Blue audience obelisk" :
                  FtcUtils.areEqual(position, OBELISK_RED_ALLIANCE_GOAL_POSITION, FtcUtils.EPSILON4) ? "Red goal obelisk" :
                      FtcUtils.areEqual(position, OBELISK_RED_ALLIANCE_AUDIENCE_POSITION, FtcUtils.EPSILON4) ? "Red audience obelisk" :
                          FtcUtils.areEqual(position, GOAL_POSITION, FtcUtils.EPSILON4) ? "Goal" : "Unknown");

      List<AprilTagDetection> detections = getAllDetections();
      if (detections != null) {
        telemetry.addData("AprilTags count", detections.size());

        // Step through the list of detections and set info for each one.
        for (AprilTagDetection detection : detections) {
          if (detection.metadata != null) {
            telemetry.addLine(String.format("Tag (ID %d) %s",
                detection.id, detection.metadata.name));
            telemetry.addLine(String.format("Range %3.1f", detection.ftcPose.range));
            telemetry.addLine(String.format("Pitch %2.1f%s roll %2.1f%s yaw %2.1f%s",
                detection.ftcPose.pitch, StringUtils.Degrees,
                detection.ftcPose.roll, StringUtils.Degrees,
                detection.ftcPose.yaw, StringUtils.Degrees));
            telemetry.addLine(String.format("Range %3.1f\" bearing %2.1f%s elevation %2.1f%s",
                detection.ftcPose.range, detection.ftcPose.bearing, StringUtils.Degrees,
                detection.ftcPose.elevation, StringUtils.Degrees));
          } else {
            telemetry.addLine(String.format("Tag (ID %d) Unknown", detection.id));
          }
        }
      } else {
        telemetry.addData("AprilTags count", 0);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    if (visionPortal != null) {
      visionPortal.resumeStreaming();
    }

    if (aprilTagServo != null) {
      // Move the servo for correct initialization.
      aprilTagServo.setPosition(Servo.MIN_POSITION);
      FtcUtils.sleep(1);
      aprilTagServo.setPosition(GOAL_POSITION);
    }

    FtcLogger.exit();
  }

  private void startAsyncOperations() {
    // Process tags on a background thread
    asyncUpdater = new FtcAprilTagAsyncUpdater(aprilTagProcessor);
    asyncUpdaterThread = new Thread(asyncUpdater, FtcAprilTagAsyncUpdater.TAG);
    asyncUpdaterThread.setPriority(Thread.NORM_PRIORITY + 1); // Priority 6 for time-sensitive vision
    asyncUpdaterThread.setDaemon(true); // Auto-terminate on shutdown for fast OpMode transitions
    asyncUpdaterThread.start();
  }

  /**
   * Stops the April Tag detection processing.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    stopAsyncOperations();
    if (visionPortal != null) {
      visionPortal.close();
      visionPortal = null;
    }

    FtcLogger.exit();
  }

  private void stopAsyncOperations() {
    if (asyncUpdater != null) {
      asyncUpdater.stop();

      // Wait for thread to finish before closing vision portal
      if (asyncUpdaterThread != null && asyncUpdaterThread.isAlive()) {
        try {
          asyncUpdaterThread.join(10); // Wait max 10ms
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }

      asyncUpdater = null;
      asyncUpdaterThread = null;
    }
  }

  /**
   * Sets camera exposure and gain.
   *
   * @param exposure The exposure to set, in milliseconds (1-1000).
   * @param gain     The gain to set (0-255).
   */
  public void updateCameraSettings(int exposure, int gain) {
    FtcLogger.enter();
    if (visionPortal != null) {
      if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
        // Ensure Manual Mode for values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
          exposureControl.setMode(ExposureControl.Mode.Manual);
        }

        exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
        currentExposure = exposure;

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        currentGain = gain;
      }
    }

    FtcLogger.exit();
  }

  public boolean isValidShootingDistance(double distance) {
    return (distance > CannonControlData.GOAL_MIN_DISTANCE && distance <= CannonControlData.GOAL_MAX_DISTANCE) ||
        (distance >= CannonControlData.AUDIENCE_MIN_DISTANCE && distance <= CannonControlData.AUDIENCE_MAX_DISTANCE);
  }
}
