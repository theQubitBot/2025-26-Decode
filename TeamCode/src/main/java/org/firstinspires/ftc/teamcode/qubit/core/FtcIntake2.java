package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot intake.
 */
public class FtcIntake2 extends FtcSubSystemBase {
  private static final String TAG = "FtcIntake2";
  public static final String INTAKE_MOTOR_NAME = "intakeMotor";
  private static final double IN_POWER = 0.50;
  private static final double OUT_POWER = -0.50;
  private static final double ZERO_POWER = 0;
  private final boolean intakeEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcMotor intakeMotor = null;

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
      intakeMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME));
      intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
      intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
      intakeMotor.setPower(IN_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Rotate outwards to outtake the object.
   */
  private void rotateOut() {
    FtcLogger.enter();
    if (intakeEnabled) {
      intakeMotor.setPower(OUT_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Displays intake telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (intakeEnabled && telemetryEnabled && intakeMotor != null) {
      telemetry.addData(TAG, String.format(Locale.US, "Position: %.4f",
          intakeMotor.getPower()));
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
   * Stops the intake.
   */
  public void stop() {
    FtcLogger.enter();
    if (intakeEnabled) {
      intakeMotor.setPower(ZERO_POWER);
    }

    FtcLogger.exit();
  }
}
