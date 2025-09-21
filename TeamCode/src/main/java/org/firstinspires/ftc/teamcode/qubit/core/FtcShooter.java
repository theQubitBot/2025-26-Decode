package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the spinning wheel.
 */
public class FtcShooter extends FtcSubSystemBase {
  private static final String TAG = "FtcShooter";
  public static final String GREEN_SHOOTER_MOTOR_NAME = "greenShooterMotor";
  public static final String PURPLE_SHOOTER_MOTOR_NAME = "purpleShooterMotor";
  public static final double MAX_POWER = 1.0;
  public static final double MIN_POWER = -1.0;

  private final boolean shooterEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcMotor greenShooterMotor = null;
  public FtcMotor purpleShooterMotor = null;

  /* Constructor */
  public FtcShooter() {
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
    if (shooterEnabled) {
      greenShooterMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, GREEN_SHOOTER_MOTOR_NAME));
      greenShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
      greenShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      greenShooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

      purpleShooterMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, PURPLE_SHOOTER_MOTOR_NAME));
      purpleShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
      purpleShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      purpleShooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the wheel using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    if (shooterEnabled) {
      double power;
      if (greenShooterMotor != null && gamePad1.xWasPressed()) {
        power = gamePad1.right_trigger;
        power = Range.clip(power, MIN_POWER, MAX_POWER);
        greenShooterMotor.setPower(power);
      } else if (purpleShooterMotor != null && gamePad1.bWasPressed()) {
        power = gamePad1.right_trigger;
        power = Range.clip(power, MIN_POWER, MAX_POWER);
        purpleShooterMotor.setPower(power);
      }
    }
  }

  /**
   * Displays wheel servo telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (shooterEnabled && telemetryEnabled) {
      telemetry.addData(TAG, String.format(Locale.US, "G: %2.1f, P: %2.1",
          greenShooterMotor.getPower(), purpleShooterMotor.getPower()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    FtcLogger.exit();
  }

  /**
   * Stops the wheel.
   */
  public void stop() {
    FtcLogger.enter();
    if (shooterEnabled) {
      greenShooterMotor.setPower(FtcMotor.ZERO_POWER);
      purpleShooterMotor.setPower(FtcMotor.ZERO_POWER);
    }

    FtcLogger.exit();
  }
}
