package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

import java.util.List;

/**
 * A class to identify and make available April Tags using the Vision Portal.
 */
public class LimelightAprilTag {
  static final String TAG = "LlAprilTag";
  public static final String APRIL_TAG_SERVO_NAME = "aprilTagServo";
  public static final String LIMELIGHT_NAME = "limelight";
  public static final double OBELISK_BLUE_ALLIANCE_GOAL_POSITION = 0.5440;
  public static final double OBELISK_BLUE_ALLIANCE_AUDIENCE_POSITION = 0.5000;
  public static final double OBELISK_RED_ALLIANCE_GOAL_POSITION = 0.4375;
  public static final double OBELISK_RED_ALLIANCE_AUDIENCE_POSITION = 0.4740;
  public static final double GOAL_POSITION = 0.4830;
  public static final double MAX_RANGE = 12000.0;
  public static final double MIN_RANGE = 0.0;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry;
  private final FtcBot parent;
  private FtcServo aprilTagServo = null;
  private Limelight3A limelight;

  public LimelightAprilTag(FtcBot robot) {
    parent = robot;
  }

  private List<LLResultTypes.FiducialResult> getAllFiducialResults() {
    List<LLResultTypes.FiducialResult> fiducialResults = null;
    if (limelight != null) {
      LLResult result = limelight.getLatestResult();
      if (result != null && result.isValid()) {
        fiducialResults = result.getFiducialResults();
      }
    }

    return fiducialResults;
  }

  public double getGoalRange() {
    double range = 0.0;
    List<LLResultTypes.FiducialResult> fiducialResults = getAllFiducialResults();
    if (fiducialResults != null && !fiducialResults.isEmpty() && parent != null && parent.config != null) {
      for (LLResultTypes.FiducialResult result : fiducialResults) {
        int tagId = result.getFiducialId();
        if ((parent.config.allianceColor == AllianceColorEnum.BLUE && tagId == 20) ||
            (parent.config.allianceColor == AllianceColorEnum.RED && tagId == 24)) {
          range = result.getCameraPoseTargetSpace().getPosition().y;
          break;
        }
      }
    }

    range = com.qualcomm.robotcore.util.Range.clip(range, MIN_RANGE, MAX_RANGE);
    return range;
  }

  public ObeliskTagEnum getObeliskTag() {
    ObeliskTagEnum ote = ObeliskTagEnum.UNKNOWN;
    List<LLResultTypes.FiducialResult> fiducialResults = getAllFiducialResults();
    if (fiducialResults != null && !fiducialResults.isEmpty()) {
      for (LLResultTypes.FiducialResult result : fiducialResults) {
        int tagId = result.getFiducialId();
        if (tagId == 21) {
          ote = ObeliskTagEnum.GPP;
        } else if (tagId == 22) {
          ote = ObeliskTagEnum.PGP;
        } else if (tagId == 23) {
          ote = ObeliskTagEnum.PPG;
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

    limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
    limelight.pipelineSwitch(0);
    limelight.start();

    aprilTagServo = new FtcServo(hardwareMap.get(Servo.class, APRIL_TAG_SERVO_NAME));
    aprilTagServo.setDirection(Servo.Direction.FORWARD);

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
    if (telemetryEnabled) {
      double position = aprilTagServo.getPosition();
      telemetry.addData(TAG, "Servo %5.4f (%s)",
          position,
          position == OBELISK_BLUE_ALLIANCE_GOAL_POSITION ? "Blue LT obelisk" :
              position == OBELISK_BLUE_ALLIANCE_AUDIENCE_POSITION ? "Blue ST obelisk" :
                  position == OBELISK_RED_ALLIANCE_GOAL_POSITION ? "Red LT obelisk" :
                      position == OBELISK_RED_ALLIANCE_AUDIENCE_POSITION ? "Red ST obelisk" :
                          position == GOAL_POSITION ? "Goal" : "Unknown");

      // Step through the list of detections and set info for each one.
      List<LLResultTypes.FiducialResult> fiducialResults = getAllFiducialResults();
      if (fiducialResults != null && !fiducialResults.isEmpty()) {
        telemetry.addData("AprilTags count", fiducialResults.size());
        for (LLResultTypes.FiducialResult result : fiducialResults) {
          Position tagPosition = result.getTargetPoseRobotSpace().getPosition();
          telemetry.addData("Tag", String.format("ID %d, x: %.1f y: %.1f",
              result.getFiducialId(), tagPosition.x, tagPosition.y));
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
  public void start() {
    FtcLogger.enter();
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
    if (limelight != null) {
      limelight.stop();
    }

    FtcLogger.exit();
  }
}
