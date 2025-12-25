package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

/**
 * A class to manage the game element intake.
 */
public class FtcIntake extends FtcSubSystemBase {
  private static final String TAG = "FtcIntake";
  public static final String LEFT_INTAKE_MOTOR_NAME = "leftIntakeMotor";
  public static final String RIGHT_INTAKE_MOTOR_NAME = "rightIntakeMotor";
  public static final double INTAKE_POWER = 0.99;
  public static final double OUTTAKE_POWER = -INTAKE_POWER;
  public static final double HOLD_POWER = 0.30;
  public static final double STOP_POWER = FtcMotor.ZERO_POWER;
  public static final long ARTIFACT_INTAKE_TIME = 1000; // milliseconds
  public static final long ARTIFACT_OUTTAKE_TIME = 1000; // milliseconds
  public static final String LIGHT_SERVO_NAME = "lightServo";
  public static final double LIGHTS_ON = 0.5; // Single light works with 0.5 value
  public static final double LIGHTS_OFF = 0.0;

  private final boolean intakeEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final BaseBot parent;
  public FtcMotor leftIntakeMotor = null;
  public FtcMotor rightIntakeMotor = null;
  public FtcServo lightServo = null;

  public FtcIntake(BaseBot robot) {
    parent = robot;
  }

  /**
   * Initialize standard Hardware interfaces.
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
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

      lightServo = new FtcServo(hardwareMap.get(Servo.class, LIGHT_SERVO_NAME));
      lightsOff();

      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  public void lightsOff() {
    lightServo.setPosition(LIGHTS_OFF);
  }

  public void lightsOn() {
    lightServo.setPosition(LIGHTS_ON);
  }

  /**
   * Operates the intake using the gamePads.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   * @param runtime  The tele op runtime.
   */
  @Override
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();
    if (intakeEnabled) {
      if (gamePad1.left_trigger >= FtcUtils.TRIGGER_THRESHOLD || gamePad2.left_trigger >= FtcUtils.TRIGGER_THRESHOLD) {
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
      lightsOn();
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
      lightsOn();
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
      lightsOff();
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
  @Override
  public void showTelemetry() {
    FtcLogger.enter();
    if (intakeEnabled && telemetryEnabled && telemetry != null) {
      if (leftIntakeMotor != null && rightIntakeMotor != null) {
        telemetry.addData(TAG, "left: %.1f, right: %.1f",
            leftIntakeMotor.getPower(), rightIntakeMotor.getPower());
      }
    }

    FtcLogger.exit();
  }

  /**
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();

    // Move the servo left and right for initialization.
    lightsOn();
    lightsOff();
    FtcLogger.exit();
  }

  /**
   * Stops the Intake.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftIntakeMotor.setPower(STOP_POWER);
      rightIntakeMotor.setPower(STOP_POWER);
      lightsOff();
    }

    FtcLogger.exit();
  }
}
