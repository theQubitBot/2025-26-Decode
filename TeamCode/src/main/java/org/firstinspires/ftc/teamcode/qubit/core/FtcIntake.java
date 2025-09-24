package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the game element intake.
 */
public class FtcIntake extends FtcSubSystemBase {
  private static final String TAG = "FtcIntake";
  public static final String LEFT_SPIN_SERVO_NAME = "leftSpinServo";
  public static final String RIGHT_SPIN_SERVO_NAME = "rightSpinServo";
  public static final String VERTICAL_SPIN_SERVO_NAME = "verticalSpinServo";
  public static final double HORIZONTAL_SPIN_IN_POWER = 0.6500;
  public static final double VERTICAL_SPIN_IN_POWER = 0.6300;
  public static final double SPIN_OUT_POWER = 0.2000;
  public static final double SPIN_HOLD_POWER = 0.5400;
  public static final double SPIN_STOP_POWER = FtcServo.MID_POSITION;
  public static final int ARTIFACT_INTAKE_TIME = 500; // milliseconds
  public static final int ARTIFACT_OUTTAKE_TIME = 1000; // milliseconds

  private final boolean intakeEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final FtcBot parent;
  private FtcServo leftSpinServo = null;
  private FtcServo rightSpinServo = null;
  private FtcServo verticalSpinServo = null;

  public FtcIntake(FtcBot robot) {
    parent = robot;
  }

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
      leftSpinServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_SPIN_SERVO_NAME));
      leftSpinServo.setDirection(Servo.Direction.REVERSE);
      rightSpinServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_SPIN_SERVO_NAME));
      rightSpinServo.setDirection(Servo.Direction.FORWARD);
      verticalSpinServo = new FtcServo(hardwareMap.get(Servo.class, VERTICAL_SPIN_SERVO_NAME));
      verticalSpinServo.setDirection(Servo.Direction.FORWARD);

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
        spinStop();
      } else if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
        spinIn(false);
      } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
        spinIn(false);
      } else if (gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
        spinOut(false);
      } else if (FtcUtils.lastNSeconds(runtime, 10)) {
        spinStop();
      } else {
        spinHold();
      }
    }

    FtcLogger.exit();
  }

  /**
   * Spin slowly inwards to hold the sample.
   */
  public void spinHold() {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftSpinServo.setPosition(SPIN_STOP_POWER);
      rightSpinServo.setPosition(SPIN_STOP_POWER);
      verticalSpinServo.setPosition(SPIN_HOLD_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Spin inwards to intake the sample.
   */
  public void spinIn(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftSpinServo.setPosition(HORIZONTAL_SPIN_IN_POWER);
      rightSpinServo.setPosition(HORIZONTAL_SPIN_IN_POWER);
      verticalSpinServo.setPosition(VERTICAL_SPIN_IN_POWER);
    }

    if (waitTillCompletion) {
      FtcUtils.sleep(ARTIFACT_INTAKE_TIME);
    }

    FtcLogger.exit();
  }

  /**
   * Spin outwards to outtake the sample.
   */
  public void spinOut(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftSpinServo.setPosition(SPIN_OUT_POWER);
      rightSpinServo.setPosition(SPIN_OUT_POWER);
      verticalSpinServo.setPosition(SPIN_OUT_POWER);
      if (waitTillCompletion) {
        FtcUtils.sleep(ARTIFACT_OUTTAKE_TIME);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Stops all motion.
   */
  public void spinStop() {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftSpinServo.setPosition(SPIN_STOP_POWER);
      rightSpinServo.setPosition(SPIN_STOP_POWER);
      verticalSpinServo.setPosition(SPIN_STOP_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Displays intake telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (intakeEnabled && telemetryEnabled) {
      if (leftSpinServo != null && rightSpinServo != null && verticalSpinServo != null) {
        telemetry.addData(TAG, String.format(Locale.US,
            "spin: %5.4f, %5.4f, %5.4f",
            leftSpinServo.getPosition(), rightSpinServo.getPosition(), verticalSpinServo.getPosition()));
      }
    }

    FtcLogger.exit();
  }

  /**
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (intakeEnabled) {
      if (leftSpinServo != null &&
          leftSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        leftSpinServo.getController().pwmEnable();
      }

      if (rightSpinServo != null &&
          rightSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        rightSpinServo.getController().pwmEnable();
      }

      if (verticalSpinServo != null &&
          verticalSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        verticalSpinServo.getController().pwmEnable();
      }

      spinStop();
    }

    FtcLogger.exit();
  }

  /**
   * Stops the Intake.
   */
  public void stop() {
    FtcLogger.enter();
    if (intakeEnabled) {
      if (leftSpinServo != null &&
          leftSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
        leftSpinServo.setPosition(SPIN_STOP_POWER);
        leftSpinServo.getController().pwmDisable();
      }

      if (rightSpinServo != null &&
          rightSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
        rightSpinServo.setPosition(SPIN_STOP_POWER);
        rightSpinServo.getController().pwmDisable();
      }

      if (verticalSpinServo != null &&
          verticalSpinServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
        verticalSpinServo.setPosition(SPIN_STOP_POWER);
        verticalSpinServo.getController().pwmDisable();
      }
    }

    FtcLogger.exit();
  }
}
