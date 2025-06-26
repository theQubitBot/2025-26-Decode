package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot intake.
 */
public class FtcServoIntake extends FtcSubSystemBase {
  private static final String TAG = "FtcIntake";
  public static final String INTAKE_SERVO_NAME = "intakeServo";
  private static final double IN_POWER = 0.60;
  private static final double OUT_POWER = 0.40;
  private static final double NO_POWER = 0.50;
  private final boolean intakeEnabled = false;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcServo intakeServo = null;

  /**
   * Initialize standard Hardware interfaces.
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    if (intakeEnabled) {
      intakeServo = new FtcServo(hardwareMap.get(Servo.class, INTAKE_SERVO_NAME));
      intakeServo.setDirection(Servo.Direction.REVERSE);
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the intake using the gamePads.
   * Left bumper -> rotate out
   * Left trigger -> rotate in
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    FtcLogger.enter();
    if (gamePad1.left_bumper || gamePad2.left_bumper) {
      rotateOut();
    } else if (gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
      rotateIn();
    } else {
      stop();
    }

    FtcLogger.exit();
  }

  /**
   * Rotate inwards to intake the object.
   */
  private void rotateIn() {
    FtcLogger.enter();
    if (intakeEnabled) {
      intakeServo.setPosition(IN_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Rotate outwards to outtake the object.
   */
  private void rotateOut() {
    FtcLogger.enter();
    if (intakeEnabled) {
      intakeServo.setPosition(OUT_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Displays intake telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (intakeEnabled && telemetryEnabled && intakeServo != null) {
      telemetry.addData(TAG, String.format(Locale.US, "Position: %.4f",
          intakeServo.getPosition()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (intakeEnabled) {
      intakeServo.getController().pwmEnable();
      intakeServo.setPosition(NO_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Stops the intake.
   */
  public void stop() {
    FtcLogger.enter();
    if (intakeEnabled) {
      intakeServo.setPosition(NO_POWER);
      intakeServo.getController().pwmDisable();
    }

    FtcLogger.exit();
  }
}
