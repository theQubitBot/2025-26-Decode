package org.firstinspires.ftc.teamcode.qubit.core;

import static org.firstinspires.ftc.teamcode.qubit.core.FtcUtils.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the spinning wheel.
 */
public class FtcSorter extends FtcSubSystemBase {
  private static final String TAG = "FtcSorter";
  public static final String SORTER_SERVO_NAME = "sorterServo";

  private final boolean sorterEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcServo sorterServo = null;
  private FtcBot parent = null;

  /* Constructor */
  public FtcSorter(FtcBot robot) {
    parent = robot;
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    if (sorterEnabled) {
      sorterServo = new FtcServo(hardwareMap.get(Servo.class, SORTER_SERVO_NAME));
      sorterServo.setDirection(Servo.Direction.FORWARD);
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the merry-go-round using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */

  // Possibly - have merry-go-round operate completely autonomously? using diff. kinds of sensors
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    FtcLogger.enter();
    if (sorterEnabled && sorterServo != null) {
    }
    FtcLogger.exit();
  }

  /**
   * Displays greenShooterMotor power telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (sorterEnabled && telemetryEnabled) {
      telemetry.addData(TAG, String.format(Locale.US, "%5.4f", sorterServo.getPosition()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();

    sleep(100);
    FtcLogger.exit();
  }

  /**
   * Stops the merry-go-round mechanism.
   */
  public void stop() {
    FtcLogger.enter();
    if (sorterEnabled && sorterServo != null &&
        sorterServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
      sorterServo.setPosition(FtcServo.MID_POSITION);
      sorterServo.getController().pwmDisable();
    }

    FtcLogger.exit();
  }
}
