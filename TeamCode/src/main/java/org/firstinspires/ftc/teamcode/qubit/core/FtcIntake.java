/* Copyright (c) 2025 The Qubit Bot. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot sample/specimen delivery.
 */
public class FtcIntake extends FtcSubSystem {
  private static final String TAG = "FtcIntake";
  public static final String LEFT_SPIN_SERVO_NAME = "leftSpinServo";
  public static final String RIGHT_SPIN_SERVO_NAME = "rightSpinServo";
  public static final String VERTICAL_SPIN_SERVO_NAME = "verticalSpinServo";
  public static final double HORIZONTAL_SPIN_IN_POWER = 0.6500;
  public static final double VERTICAL_SPIN_IN_POWER = 0.6300;
  public static final double SPIN_OUT_POWER = 0.2000;
  public static final double SPIN_HOLD_POWER = 0.5400;
  public static final double SPIN_STOP_POWER = FtcServo.MID_POSITION;
  public static final String LEFT_FLIP_SERVO_NAME = "leftFlipServo";
  public static final String RIGHT_FLIP_SERVO_NAME = "rightFlipServo";
  public static final double FLIP_DOWN_LEFT_POSITION = 0.4925;
  public static final double FLIP_DOWN_RIGHT_POSITION = 0.4955;
  public static final double FLIP_HORIZONTAL_LEFT_POSITION = 0.5345;
  public static final double FLIP_HORIZONTAL_RIGHT_POSITION = 0.5320;
  public static final double FLIP_DELIVER_LEFT_POSITION = 0.5700;
  public static final double FLIP_DELIVER_RIGHT_POSITION = 0.5655;
  public static final int FLIP_TRAVEL_TIME = 1000; // milliseconds
  public static final int SAMPLE_INTAKE_TIME = 500; // milliseconds
  public static final int SAMPLE_OUTTAKE_TIME = 1000; // milliseconds
  public static final int SPECIMEN_INTAKE_TIME = 1000; // milliseconds
  public static final String LEFT_SPECIMEN_SERVO_NAME = "leftSpecimenServo";
  public static final String RIGHT_SPECIMEN_SERVO_NAME = "rightSpecimenServo";
  public static final double LEFT_SPECIMEN_GRAB_POSITION = 0.4385;
  public static final double LEFT_SPECIMEN_RELEASE_POSITION = 0.50;
  public static final double RIGHT_SPECIMEN_GRAB_POSITION = 0.5585;
  public static final double RIGHT_SPECIMEN_RELEASE_POSITION = 0.50;

  private final boolean intakeEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  private final FtcBot parent;
  private FtcServo leftSpinServo = null;
  private FtcServo rightSpinServo = null;
  private FtcServo verticalSpinServo = null;
  private FtcServo leftFlipServo = null;
  private FtcServo rightFlipServo = null;
  private FtcServo leftSpecimenServo = null;
  private FtcServo rightSpecimenServo = null;

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

      leftFlipServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_FLIP_SERVO_NAME));
      leftFlipServo.setDirection(Servo.Direction.FORWARD);
      rightFlipServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_FLIP_SERVO_NAME));
      rightFlipServo.setDirection(Servo.Direction.REVERSE);

      leftSpecimenServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_SPECIMEN_SERVO_NAME));
      leftSpecimenServo.setDirection(Servo.Direction.FORWARD);
      rightSpecimenServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_SPECIMEN_SERVO_NAME));
      rightSpecimenServo.setDirection(Servo.Direction.FORWARD);

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
        specimenRelease();
      } else if (FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
        flipHorizontal(false);
        spinStop();
        specimenRelease();
      } else if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
        spinIn(false);
        flipDown(false);
        specimenGrab(false);
      } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
        if (
          // Both drivers force manual override
            (gamePad1.right_bumper && gamePad2.right_bumper) ||
                // GamePad1 manual override
                (gamePad1.share && gamePad1.right_bumper) ||
                // GamePad2 manual override
                (gamePad2.share && gamePad2.right_bumper) ||
                // Testing mode, parent may not be specified
                parent == null) {
          spinIn(false);
          flipDelivery(false);
        } else if (parent.lift.atPosition(FtcLift.POSITION_FLOOR) &&
            parent.arm.isForward() &&
            parent.rnp.isRetracted()) {
          // Flip to delivery when lift is Low, bucket is forward and rnp is retracted
          spinIn(false);
          flipDelivery(false);
        }

        specimenRelease();
      } else if (gamePad2.right_stick_y <= -0.5 || gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
        flipDown(false);
        spinOut(false);
      } else if (FtcUtils.lastNSeconds(runtime, 10)) {
        spinStop();
        flipHorizontal(false);
      } else {
        spinHold();
        flipHorizontal(false);
      }
    }

    FtcLogger.exit();
  }

  public boolean isDelivering() {
    FtcLogger.enter();
    boolean delivering = false;
    if (intakeEnabled) {
      delivering = FtcUtils.areEqual(leftFlipServo.getPosition(), FLIP_DELIVER_LEFT_POSITION, FtcUtils.EPSILON3);
    }

    FtcLogger.exit();
    return delivering;
  }

  public boolean isDown() {
    FtcLogger.enter();
    boolean down = false;
    if (intakeEnabled) {
      down = FtcUtils.areEqual(leftFlipServo.getPosition(), FLIP_DOWN_LEFT_POSITION, FtcUtils.EPSILON3);
    }

    FtcLogger.exit();
    return down;
  }

  public boolean isHorizontal() {
    FtcLogger.enter();
    boolean horizontal = false;
    if (intakeEnabled) {
      horizontal = FtcUtils.areEqual(leftFlipServo.getPosition(), FLIP_HORIZONTAL_LEFT_POSITION, FtcUtils.EPSILON3);
    }

    FtcLogger.exit();
    return horizontal;
  }

  public void flipDelivery(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftFlipServo.setPosition(FLIP_DELIVER_LEFT_POSITION);
      rightFlipServo.setPosition(FLIP_DELIVER_RIGHT_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(FLIP_TRAVEL_TIME);
      }
    }

    FtcLogger.exit();
  }

  public void flipHorizontal(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftFlipServo.setPosition(FLIP_HORIZONTAL_LEFT_POSITION);
      rightFlipServo.setPosition(FLIP_HORIZONTAL_RIGHT_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(FLIP_TRAVEL_TIME);
      }
    }

    FtcLogger.exit();
  }

  public void flipDown(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      leftFlipServo.setPosition(FLIP_DOWN_LEFT_POSITION);
      rightFlipServo.setPosition(FLIP_DOWN_RIGHT_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(FLIP_TRAVEL_TIME);
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
      leftSpinServo.setPosition(SPIN_HOLD_POWER);
      rightSpinServo.setPosition(SPIN_HOLD_POWER);
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
      FtcUtils.sleep(SAMPLE_INTAKE_TIME);
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
        FtcUtils.sleep(SAMPLE_OUTTAKE_TIME);
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
      if (leftSpinServo != null && rightSpinServo != null && verticalSpinServo != null &&
          leftFlipServo != null && rightFlipServo != null) {
        telemetry.addData(TAG, String.format(Locale.US,
            "spin: %5.4f, %5.4f, %5.4f, flip: %5.4f, %5.4f",
            leftSpinServo.getPosition(), rightSpinServo.getPosition(), verticalSpinServo.getPosition(),
            leftFlipServo.getPosition(), rightFlipServo.getPosition()));
      }
    }

    FtcLogger.exit();
  }

  public void leftSpecimenGrab(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled && leftSpecimenServo != null) {
      leftSpecimenServo.setPosition(LEFT_SPECIMEN_GRAB_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SPECIMEN_INTAKE_TIME);
      }
    }

    FtcLogger.exit();
  }

  public void leftSpecimenRelease() {
    FtcLogger.enter();
    if (intakeEnabled && leftSpecimenServo != null) {
      leftSpecimenServo.setPosition(LEFT_SPECIMEN_RELEASE_POSITION);
    }

    FtcLogger.exit();
  }

  public void rightSpecimenGrab(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled && rightSpecimenServo != null) {
      rightSpecimenServo.setPosition(RIGHT_SPECIMEN_GRAB_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SPECIMEN_INTAKE_TIME);
      }
    }

    FtcLogger.exit();
  }

  public void rightSpecimenRelease() {
    FtcLogger.enter();
    if (intakeEnabled && rightSpecimenServo != null) {
      rightSpecimenServo.setPosition(RIGHT_SPECIMEN_RELEASE_POSITION);
    }

    FtcLogger.exit();
  }

  public void specimenGrab(boolean waitTillCompletion) {
    FtcLogger.enter();
    if (intakeEnabled) {
      if (leftSpecimenServo != null) {
        leftSpecimenServo.setPosition(LEFT_SPECIMEN_GRAB_POSITION);
      }

      if (rightSpecimenServo != null) {
        rightSpecimenServo.setPosition(RIGHT_SPECIMEN_GRAB_POSITION);
      }

      if (waitTillCompletion) {
        FtcUtils.sleep(SPECIMEN_INTAKE_TIME);
      }
    }

    FtcLogger.exit();
  }

  public void specimenRelease() {
    FtcLogger.enter();
    if (intakeEnabled) {
      if (leftSpecimenServo != null) {
        leftSpecimenServo.setPosition(LEFT_SPECIMEN_RELEASE_POSITION);
      }

      if (rightSpecimenServo != null) {
        rightSpecimenServo.setPosition(RIGHT_SPECIMEN_RELEASE_POSITION);
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

      if (leftFlipServo != null &&
          leftFlipServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        leftFlipServo.getController().pwmEnable();
      }

      if (rightFlipServo != null &&
          rightFlipServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        rightFlipServo.getController().pwmEnable();
      }

      spinStop();
      flipHorizontal(false);
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

      if (leftFlipServo != null &&
          leftFlipServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
        leftFlipServo.setPosition(SPIN_STOP_POWER);
        leftFlipServo.getController().pwmDisable();
      }

      if (rightFlipServo != null &&
          rightFlipServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
        rightFlipServo.setPosition(SPIN_STOP_POWER);
        rightFlipServo.getController().pwmDisable();
      }
    }

    FtcLogger.exit();
  }
}
