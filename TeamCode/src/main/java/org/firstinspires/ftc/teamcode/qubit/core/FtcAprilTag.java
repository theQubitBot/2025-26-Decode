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
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * A class to identify and make available April Tags using the Vision Portal.
 */
public class FtcAprilTag {
  static final String TAG = "FtcAprilTag";
  public static final String APRIL_TAG_SERVO_NAME = "aprilTagServo";
  public static final double OBELISK_BLUE_ALLIANCE_GOAL_POSITION = 0.5350;
  public static final double OBELISK_BLUE_ALLIANCE_AUDIENCE_POSITION = 0.5020;
  public static final double OBELISK_RED_ALLIANCE_GOAL_POSITION = 0.4395;
  public static final double OBELISK_RED_ALLIANCE_AUDIENCE_POSITION = 0.4700;
  public static final double GOAL_POSITION = 0.4830;
  public static final double MAX_RANGE = 120.0;
  public static final double MIN_RANGE = 0.0;
  public static final int CAMERA_EXPOSURE = 10; // milliseconds
  public static final int CAMERA_GAIN = 40;
  private AprilTagProcessor aprilTagProcessor;
  private VisionPortal visionPortal;
  private FtcAprilTagAsyncUpdater updater;

  public int currentExposure, minExposure, maxExposure;
  public int currentGain, minGain, maxGain;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry;
  private final FtcBot parent;
  private FtcServo aprilTagServo = null;

  public FtcAprilTag(FtcBot robot) {
    parent = robot;
  }

  private List<AprilTagDetection> getAllDetections() {
    List<AprilTagDetection> detections = updater.getAllDetections();
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

  public ObeliskTagEnum getObeliskTag() {
    ObeliskTagEnum ote = ObeliskTagEnum.UNKNOWN;
    List<AprilTagDetection> detections = getAllDetections();
    if (detections != null && !detections.isEmpty()) {
      for (AprilTagDetection detection : detections) {
        if (detection.metadata != null) {
          if (detection.id == 21) {
            ote = ObeliskTagEnum.GPP;
          } else if (detection.id == 22) {
            ote = ObeliskTagEnum.PGP;
          } else if (detection.id == 23) {
            ote = ObeliskTagEnum.PPG;
          }
        }
      }
    }

    return ote;
  }

  /**
   * Initialize the AprilTag detection processor.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
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
        .setCamera(hardwareMap.get(WebcamName.class, FtcOpenCvCam.WEBCAM_2_NAME))
        .setCameraResolution(new Size(FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT))
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

    // Process tags on a background thread
    updater = new FtcAprilTagAsyncUpdater(aprilTagProcessor);
    Thread updaterThread = new Thread(updater);
    updaterThread.start();

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
    if (telemetryEnabled && aprilTagProcessor != null) {
      double position = aprilTagServo.getPosition();
      telemetry.addData(TAG, "Servo %5.4f (%s)",
          position,
          position == OBELISK_BLUE_ALLIANCE_GOAL_POSITION ? "Blue goal obelisk" :
              position == OBELISK_BLUE_ALLIANCE_AUDIENCE_POSITION ? "Blue audience obelisk" :
                  position == OBELISK_RED_ALLIANCE_GOAL_POSITION ? "Red goal obelisk" :
                      position == OBELISK_RED_ALLIANCE_AUDIENCE_POSITION ? "Red audience obelisk" :
                          position == GOAL_POSITION ? "Goal" : "Unknown");
      List<AprilTagDetection> aprilTagDetections = getAllDetections();
      telemetry.addData("AprilTags count", aprilTagDetections.size());

      // Step through the list of detections and set info for each one.
      for (AprilTagDetection detection : aprilTagDetections) {
        if (detection.metadata != null) {
          telemetry.addLine(String.format("Tag (ID %d) %s",
              detection.id, detection.metadata.name));
          telemetry.addLine(String.format("Range %6.1f", detection.ftcPose.range));
        } else {
          telemetry.addLine(String.format("Tag (ID %d) Unknown", detection.id));
        }
      }
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
}
