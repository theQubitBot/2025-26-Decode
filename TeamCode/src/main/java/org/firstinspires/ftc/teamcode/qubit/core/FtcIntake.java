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
  public static final String LEFT_ROLLER_SERVO_NAME = "leftRollerServo";
  public static final String RIGHT_ROLLER_SERVO_NAME = "rightRollerServo";
  public static final String LEFT_SWEEPER_SERVO_NAME = "leftSweeperServo";
  public static final String RIGHT_SWEEPER_SERVO_NAME = "rightSweeperServo";
  public static final double ROLLER_IN_POWER = 1.0000;
  public static final double ROLLER_OUT_POWER = 0.1000;
  public static final double SWEEPER_IN_POWER = 1.000;
  public static final double SWEEPER_OUT_POWER = 0.1000;
  public static final double ROLLER_HOLD_POWER = 0.5400;
  public static final double SWEEPER_HOLD_POWER = 0.5000;
  public static final double INTAKE_STOP_POWER = FtcServo.MID_POSITION;
  public static final int ARTIFACT_INTAKE_TIME = 1000; // milliseconds
  public static final int ARTIFACT_OUTTAKE_TIME = 1000; // milliseconds

  private final boolean intakeEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private FtcServo leftRollerServo = null;
  private FtcServo rightRollerServo = null;
  private FtcServo leftSweeperServo = null;
  private FtcServo rightSweeperServo = null;

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
      leftRollerServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_ROLLER_SERVO_NAME));
      leftRollerServo.setDirection(Servo.Direction.REVERSE);
      rightRollerServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_ROLLER_SERVO_NAME));
      rightRollerServo.setDirection(Servo.Direction.FORWARD);
      leftSweeperServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_SWEEPER_SERVO_NAME));
      leftSweeperServo.setDirection(Servo.Direction.FORWARD);
      rightSweeperServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_SWEEPER_SERVO_NAME));
      rightSweeperServo.setDirection(Servo.Direction.REVERSE);

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

  /**
   * Spin slowly inwards to hold the artifact.
   */
  public void spinHold() {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftRollerServo.setPosition(ROLLER_HOLD_POWER);
      rightRollerServo.setPosition(ROLLER_HOLD_POWER);
      leftSweeperServo.setPosition(SWEEPER_HOLD_POWER);
      rightSweeperServo.setPosition(SWEEPER_HOLD_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Spin inwards to intake the artifact.
   */
  public void spinIn(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftRollerServo.setPosition(ROLLER_IN_POWER);
      rightRollerServo.setPosition(ROLLER_IN_POWER);
      leftSweeperServo.setPosition(SWEEPER_IN_POWER);
      rightSweeperServo.setPosition(SWEEPER_IN_POWER);
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
      leftRollerServo.setPosition(ROLLER_OUT_POWER);
      rightRollerServo.setPosition(ROLLER_OUT_POWER);
      leftSweeperServo.setPosition(SWEEPER_OUT_POWER);
      rightSweeperServo.setPosition(SWEEPER_OUT_POWER);
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
      leftRollerServo.setPosition(INTAKE_STOP_POWER);
      rightRollerServo.setPosition(INTAKE_STOP_POWER);
      leftSweeperServo.setPosition(INTAKE_STOP_POWER);
      rightSweeperServo.setPosition(INTAKE_STOP_POWER);
    }

    FtcLogger.exit();
  }

  /**
   * Displays intake telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (intakeEnabled && telemetryEnabled) {
      if (leftRollerServo != null && rightRollerServo != null
          && leftSweeperServo != null && rightSweeperServo != null) {
        telemetry.addData(TAG, String.format(Locale.US,
            "roller: %5.4f, %5.4f, sweeper: %5.4f, %5.4f",
            leftRollerServo.getPosition(), rightRollerServo.getPosition(),
            leftSweeperServo.getPosition(), rightSweeperServo.getPosition()));
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
      if (leftRollerServo != null) {
        if (leftRollerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
          leftRollerServo.getController().pwmEnable();
        }

        leftRollerServo.setPosition(INTAKE_STOP_POWER);
      }

      if (rightRollerServo != null) {
        if (rightRollerServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
          rightRollerServo.getController().pwmEnable();
        }

        rightRollerServo.setPosition(INTAKE_STOP_POWER);
      }

      if (leftSweeperServo != null) {
        if (leftSweeperServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
          leftSweeperServo.getController().pwmEnable();
        }

        leftSweeperServo.setPosition(INTAKE_STOP_POWER);
      }

      if (rightSweeperServo != null) {
        if (rightSweeperServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
          rightSweeperServo.getController().pwmEnable();
        }

        rightSweeperServo.setPosition(INTAKE_STOP_POWER);
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
      if (leftRollerServo != null) {
        leftRollerServo.setPosition(INTAKE_STOP_POWER);
      }

      if (rightRollerServo != null) {
        rightRollerServo.setPosition(INTAKE_STOP_POWER);
      }

      if (leftSweeperServo != null) {
        leftSweeperServo.setPosition(INTAKE_STOP_POWER);
      }

      if (rightSweeperServo != null) {
        rightSweeperServo.setPosition(INTAKE_STOP_POWER);
      }
    }

    FtcLogger.exit();
  }
}
