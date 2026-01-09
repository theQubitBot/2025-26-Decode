package org.firstinspires.ftc.teamcode.qubit.core;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.qubit.core.Field.FtcField;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

import java.io.File;
import java.io.IOException;

/**
 * A configuration class to hold match configuration data.
 * Configuration is saved to a file and persists across
 * auto ops, tele ops and reboots.
 */
public class MatchConfig {
  private static final String TAG = "MatchConfig";
  public static final String MATCH_CONFIG_FILENAME = "matchConfig.txt";
  private static final long MAX_START_DELAY_SECONDS = 20;
  private boolean gamePad1Connected, gamePad2Connected;
  public boolean configIsComplete;
  private final boolean configFeatureEnabled = true;
  private Telemetry telemetry = null;

  public AllianceColorEnum allianceColor;
  public RobotPositionEnum robotPosition;
  public long delayInSeconds;
  public ObeliskTagEnum obeliskTagEnum;
  public boolean deliverSecondRow = true;
  public boolean deliverThirdRow = true;

  // Robot pose information
  public double x, y, heading; // radians

  /**
   * Constructor
   */
  public MatchConfig() {
    if (configFeatureEnabled) {
      reset();
    }
  }

  /**
   * Initialize standard Hardware interfaces.
   * Reads match configuration from the configuration file.
   *
   * @param hardwareMap Not used.
   * @param telemetry   The telemetry to use.
   * @param autoOp      Not used.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    if (configFeatureEnabled) {
      readFromFile();
      if (autoOp) {
        Pose p = new Pose(0, 0, 0);
        if (allianceColor == AllianceColorEnum.BLUE) {
          if (robotPosition == RobotPositionEnum.GOAL) {
            p = FtcField.blueGoalStartPose;
          } else if (robotPosition == RobotPositionEnum.AUDIENCE) {
            p = FtcField.blueAudienceStartPose;
          }
        } else if (allianceColor == AllianceColorEnum.RED) {
          if (robotPosition == RobotPositionEnum.GOAL) {
            p = FtcField.redGoalStartPose;
          } else if (robotPosition == RobotPositionEnum.AUDIENCE) {
            p = FtcField.redAudienceStartPose;
          }
        }

        x = p.getX();
        y = p.getY();
        heading = p.getHeading();
      } else {
        // use the robot position saved from autoOp
      }
    }

    FtcLogger.exit();
  }

  /**
   * Determines if the gamePad is connected to the driver station.
   *
   * @param gamePad The gamePad to evaluate.
   * @return true, if gamePad is connected, false otherwise.
   */
  private boolean isGamePadConnected(Gamepad gamePad) {
    // GamePad is assumed to be connected if any of the inputs
    // is not a default input.
    return gamePad.getGamepadId() != Gamepad.ID_UNASSOCIATED || gamePad.getUser() != null ||
        gamePad.a || gamePad.b || gamePad.x || gamePad.y ||
        gamePad.left_bumper || gamePad.left_trigger > 0.1 ||
        gamePad.right_bumper || gamePad.right_trigger > 0.1 ||
        gamePad.dpad_left || gamePad.dpad_right || gamePad.dpad_up || gamePad.dpad_down ||
        Math.abs(gamePad.left_stick_x) > 0.1 || Math.abs(gamePad.left_stick_y) > 0.1 ||
        Math.abs(gamePad.right_stick_x) > 0.1 || Math.abs(gamePad.right_stick_y) > 0.1 ||
        gamePad.left_stick_button || gamePad.right_stick_button;
  }

  /**
   * Reads match configuration from the saved configuration file.
   */
  private void readFromFile() {
    FtcLogger.enter();

    try {
      File file = AppUtil.getInstance().getSettingsFile(MATCH_CONFIG_FILENAME);
      String data = ReadWriteFile.readFileOrThrow(file);
      MatchConfig savedMatchConfig = FtcUtils.deserialize(data, MatchConfig.class);
      allianceColor = savedMatchConfig.allianceColor;
      robotPosition = savedMatchConfig.robotPosition;
      delayInSeconds = savedMatchConfig.delayInSeconds;
      obeliskTagEnum = savedMatchConfig.obeliskTagEnum;
      deliverSecondRow = savedMatchConfig.deliverSecondRow;
      deliverThirdRow = savedMatchConfig.deliverThirdRow;
      x = savedMatchConfig.x;
      y = savedMatchConfig.y;
      heading = savedMatchConfig.heading;
    } catch (IOException e) {
      reset();
    } catch (Exception e) {
      // First time around, config file doesn't exist. Do nothing
      reset();
    }

    FtcLogger.exit();
  }

  /**
   * Resets the match configuration to default values.
   */
  public void reset() {
    FtcLogger.enter();
    allianceColor = AllianceColorEnum.RED;
    robotPosition = RobotPositionEnum.AUDIENCE;
    delayInSeconds = 0;
    obeliskTagEnum = ObeliskTagEnum.GPP;
    deliverSecondRow = true;
    deliverThirdRow = true;
    x = y = heading = 0;
    gamePad1Connected = gamePad2Connected = false;
    configIsComplete = !configFeatureEnabled;
    FtcLogger.exit();
  }

  /**
   * Updates match configuration based on gamPad inputs.
   *
   * @param gamePad1 The first gamPad to use.
   * @param gamePad2 The second gamPad to use.
   */
  public void update(Gamepad gamePad1, Gamepad gamePad2) {
    if (configFeatureEnabled) {
      // Configure alliance color
      if (gamePad1.rightBumperWasPressed() || gamePad1.leftBumperWasPressed() ||
          gamePad2.rightBumperWasPressed() || gamePad2.leftBumperWasPressed()) {
        allianceColor = AllianceColorEnum.BLUE;
      } else if (gamePad1.right_trigger > 0.5 || gamePad1.left_trigger > 0.5 ||
          gamePad2.right_trigger > 0.5 || gamePad2.left_trigger > 0.5) {
        allianceColor = AllianceColorEnum.RED;
      }

      // Configure robot position on the field
      if (gamePad1.dpadLeftWasPressed() || gamePad2.dpadLeftWasPressed()) {
        robotPosition = RobotPositionEnum.GOAL;
      } else if (gamePad1.dpadRightWasPressed() || gamePad2.dpadRightWasPressed()) {
        robotPosition = RobotPositionEnum.AUDIENCE;
      }

      // Configure initial delay
      if (gamePad1.dpadUpWasPressed() || gamePad2.dpadUpWasPressed()) {
        delayInSeconds = Math.min(delayInSeconds + 1, MAX_START_DELAY_SECONDS);
      } else if (gamePad1.dpadDownWasPressed() || gamePad2.dpadDownWasPressed()) {
        delayInSeconds = Math.max(delayInSeconds - 1, 0);
      }

      // Configure extra row delivery
      if (gamePad1.circleWasPressed() || gamePad2.circleWasPressed()) {
        deliverSecondRow = !deliverSecondRow;
      } else if (gamePad1.crossWasPressed() || gamePad2.crossWasPressed()) {
        deliverThirdRow = !deliverThirdRow;
      }

      // Evaluate gamePad connections, if not connected.
      if (!gamePad1Connected) {
        gamePad1Connected = isGamePadConnected(gamePad1);
      }

      if (!gamePad2Connected) {
        gamePad2Connected = isGamePadConnected(gamePad2);
      }
    }
  }

  /**
   * Saves the configuration to the configuration file.
   */
  public void saveToFile() {
    FtcLogger.enter();
    if (configFeatureEnabled) {
      File file = AppUtil.getInstance().getSettingsFile(MATCH_CONFIG_FILENAME);
      ReadWriteFile.writeFile(file, FtcUtils.serialize(this));
    }

    FtcLogger.exit();
  }

  /**
   * Displays Match Configuration on Driver Station screen.
   */
  public void showConfiguration() {
    if (configFeatureEnabled && telemetry != null) {
      telemetry.addData("Match configuration", "");
      telemetry.addData(FtcUtils.TAG, "%s alliance, %s position",
          allianceColor, robotPosition);
      telemetry.addData(FtcUtils.TAG, "start delay %d seconds", delayInSeconds);
      telemetry.addData(FtcUtils.TAG, "deliver 2nd row: %s", deliverSecondRow ? "yes" : "no");
      telemetry.addData(FtcUtils.TAG, "deliver 3rd row: %s", deliverThirdRow ? "yes" : "no");
      telemetry.addData(FtcUtils.TAG, "Motif %s", obeliskTagEnum);
      telemetry.addData(FtcUtils.TAG, "Robot pose (%.1f, %.1f, %.1f)", x, y, Math.toDegrees(heading));
    }
  }

  /**
   * Displays Match Configuration Command Menu on Driver Station screen.
   */
  public void showMenu() {
    if (configFeatureEnabled && telemetry != null) {
      telemetry.addData("Match Configuration Command Menu", "");
      telemetry.addData("Bumper", "BLUE alliance");
      telemetry.addData("Trigger", "RED alliance");
      telemetry.addData("dPad", "left: GOAL, right: AUDIENCE position");
      telemetry.addData("Start delay", "dPad up/down: +/-");
      telemetry.addData("Deliver 2nd row", "%s: toggle yes/no",
          StringUtils.CircleOutline);
      telemetry.addData("Deliver 3rd row", "%s: toggle yes/no",
          StringUtils.Cross);
    }
  }
}
