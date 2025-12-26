package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;

/**
 * A class to manage the REV Blinkin LED strip.
 */
public class FtcBlinkinLed extends FtcSubSystemBase {
  private static final String TAG = "FtcBlinkinLed";
  private static final String BLINKIN_NAME = "blinkinLedDriver";
  private final boolean blinkinLedEnabled = true;
  public boolean telemetryEnabled = true;
  private RevBlinkinLedDriver blinkinLedDriver = null;

  // Start with LED strip being off.
  private BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
  private BlinkinPattern parkingPattern;
  private Telemetry telemetry;
  private final BaseBot parent;

  public FtcBlinkinLed(BaseBot robot) {
    parent = robot;
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    this.telemetry = telemetry;
    if (blinkinLedEnabled) {
      blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, BLINKIN_NAME);
      set(currentPattern);

      if (parent.config != null) {
        if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
          parkingPattern = BlinkinPattern.HEARTBEAT_BLUE;
        } else {
          parkingPattern = BlinkinPattern.HEARTBEAT_RED;
        }
      } else {
        parkingPattern = BlinkinPattern.HEARTBEAT_WHITE;
      }

      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }
  }

  /**
   * Checks if cannon is ready to fire at the target distance and sets LED accordingly.
   *
   * @param distance The target distance to check cannon readiness for.
   */
  private void checkCannonReadyAndIndicate(double distance) {
    if (parent.cannon != null) {
      if (parent.cannon.isPrimed(distance)) {
        set(BlinkinPattern.GREEN);
      } else {
        stop();
      }
    } else {
      stop();
    }
  }

  /**
   * Set a LED pattern based on gameplay conditions.
   *
   * @param gamePad1 Gamepad1 to use.
   * @param gamePad2 Gamepad2 to use.
   * @param runtime  The cumulative time game has been running.
   */
  @Override
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();
    if (gamePad1.right_bumper || gamePad2.right_bumper) {
      checkCannonReadyAndIndicate(FtcCannon.AUDIENCE_DISTANCE);
    } else if (gamePad1.right_trigger >= FtcUtils.TRIGGER_THRESHOLD || gamePad2.right_trigger >= FtcUtils.TRIGGER_THRESHOLD) {
      checkCannonReadyAndIndicate(FtcCannon.GOAL_SWEET_SPOT_DISTANCE);
    } else if (FtcUtils.lastNSeconds(runtime, FtcUtils.ENDGAME_PARK_WARNING_SECONDS)) {
      set(parkingPattern);
    } else if (parent.localizer != null && parent.localizer.robotWithinLaunchZone() &&
        parent.localizer.robotPointingAtGoal()) {
      checkCannonReadyAndIndicate(parent.localizer.getGoalDistance());
    } else {
      stop();
    }

    FtcLogger.exit();
  }

  /**
   * Set the next pattern.
   */
  public void setNextPattern() {
    set(currentPattern.next());
  }

  /**
   * Set the previous pattern.
   */
  public void setPreviousPattern() {
    set(currentPattern.previous());
  }

  /**
   * Sets the LED display pattern.
   *
   * @param pattern The display pattern to set to.
   */
  private void set(RevBlinkinLedDriver.BlinkinPattern pattern) {
    if (blinkinLedEnabled && blinkinLedDriver != null) {
      if (currentPattern != pattern) {
        currentPattern = pattern;
        blinkinLedDriver.setPattern(currentPattern);
      }
    }
  }

  /**
   * Displays LED telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    if (telemetryEnabled && telemetry != null) {
      telemetry.addData(TAG, "%s (%d)",
          currentPattern.toString(), currentPattern.ordinal());
    }
  }

  /**
   * Starts the current LED display pattern by setting the BLACK pattern.
   */
  @Override
  public void start() {
    // Change the LED position for proper initialization.
    set(BlinkinPattern.WHITE);
    set(BlinkinPattern.BLACK);
  }

  /**
   * Stops the current LED display pattern by setting the BLACK pattern.
   */
  @Override
  public void stop() {
    set(BlinkinPattern.BLACK);
  }
}
