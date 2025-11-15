package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the game element intake.
 */
public class FtcIntake extends FtcSubSystemBase {
  private static final String TAG = "FtcIntake";
  public static final String LEFT_INTAKE_MOTOR_NAME = "leftIntakeMotor";
  public static final String RIGHT_INTAKE_MOTOR_NAME = "rightIntakeMotor";
  public static final double INTAKE_POWER = 0.80;
  public static final double OUTTAKE_POWER = -0.50;
  public static final double HOLD_POWER = 0.25;
  public static final double STOP_POWER = FtcMotor.ZERO_POWER;
  public static final long ARTIFACT_INTAKE_TIME = 1000; // milliseconds
  public static final long ARTIFACT_OUTTAKE_TIME = 1000; // milliseconds

  private final boolean intakeEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcMotor leftIntakeMotor = null;
  public FtcMotor rightIntakeMotor = null;

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
      leftIntakeMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, LEFT_INTAKE_MOTOR_NAME));
      leftIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
      leftIntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      leftIntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

      rightIntakeMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_INTAKE_MOTOR_NAME));
      rightIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
      rightIntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      rightIntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the intake using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   * @param runtime  The tele op runtime.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
    FtcLogger.enter();
    if (intakeEnabled) {
      if (!FtcUtils.DEBUG && FtcUtils.gameOver(runtime)) {
        stop();
      } else if (gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
        spinIn(false);
      } else if (gamePad1.left_bumper || gamePad2.left_bumper) {
        spinOut(false);
      } else {
        spinHold();
      }
    }

    FtcLogger.exit();
  }

  public void setPower(double power) {
    leftIntakeMotor.setPower(power);
    rightIntakeMotor.setPower(power);
  }

  /**
   * Spin slowly inwards to hold the artifact.
   */
  public void spinHold() {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftIntakeMotor.setPower(HOLD_POWER);
      rightIntakeMotor.setPower(HOLD_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Spin inwards to intake the artifact.
   */
  public void spinIn(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftIntakeMotor.setPower(INTAKE_POWER);
      rightIntakeMotor.setPower(INTAKE_POWER);
      if (waitTillCompletion) {
        FtcUtils.sleep(ARTIFACT_INTAKE_TIME);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Spin outwards to outtake the artifact.
   */
  public void spinOut(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftIntakeMotor.setPower(OUTTAKE_POWER);
      rightIntakeMotor.setPower(OUTTAKE_POWER);
      if (waitTillCompletion) {
        FtcUtils.sleep(ARTIFACT_OUTTAKE_TIME);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Displays intake telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (intakeEnabled && telemetryEnabled) {
      if (leftIntakeMotor != null && rightIntakeMotor != null) {
        telemetry.addData(TAG, String.format(Locale.US,
            "left: %.1f, right: %.1f",
            leftIntakeMotor.getPower(), rightIntakeMotor.getPower()));
      }
    }

    FtcLogger.exit();
  }

  /**
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    FtcLogger.exit();
  }

  /**
   * Stops the Intake.
   */
  public void stop() {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftIntakeMotor.setPower(STOP_POWER);
      rightIntakeMotor.setPower(STOP_POWER);
    }

    FtcLogger.exit();
  }
}
