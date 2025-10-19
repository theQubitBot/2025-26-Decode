package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;

import java.util.Locale;

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
  private Telemetry telemetry;
  private final FtcBot parent;

  public FtcBlinkinLed(FtcBot robot) {
    parent = robot;
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;
    if (blinkinLedEnabled) {
      blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, BLINKIN_NAME);
      set(currentPattern);
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }
  }

  /**
   * Set a LED pattern based on gameplay conditions.
   *
   * @param gamePad1 Gamepad1 to use.
   * @param gamePad2 Gamepad2 to use.
   * @param runtime  The cumulative time game has been running.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
    FtcLogger.enter();
    if (!FtcUtils.DEBUG && FtcUtils.gameOver(runtime)) {
      stop();
    } else if (gamePad1.x || gamePad2.x || gamePad1.b || gamePad2.b) {
      set(BlinkinPattern.TWINKLES_LAVA_PALETTE);
    } else if (parent != null && parent.aprilTag != null && parent.aprilTag.getGoalRange() > 0) {
      set(BlinkinPattern.GREEN);
    } else if (FtcUtils.lastNSeconds(runtime, 10)) {
      if (parent != null) {
        if (parent.config.allianceColor == AllianceColorEnum.BLUE) {
          set(BlinkinPattern.HEARTBEAT_BLUE);
        } else {
          set(BlinkinPattern.HEARTBEAT_RED);
        }
      } else {
        set(BlinkinPattern.HEARTBEAT_WHITE);
      }
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
  public void set(RevBlinkinLedDriver.BlinkinPattern pattern) {
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
    if (blinkinLedEnabled && telemetryEnabled && blinkinLedDriver != null) {
      telemetry.addData(TAG, String.format(Locale.US, "%s (%d)",
          currentPattern.toString(), currentPattern.ordinal()));
    }
  }

  /**
   * Starts the current LED display pattern by setting the BLACK pattern.
   */
  public void start() {
    set(BlinkinPattern.BLACK);
  }

  /**
   * Stops the current LED display pattern by setting the BLACK pattern.
   */
  public void stop() {
    set(BlinkinPattern.BLACK);
  }
}
