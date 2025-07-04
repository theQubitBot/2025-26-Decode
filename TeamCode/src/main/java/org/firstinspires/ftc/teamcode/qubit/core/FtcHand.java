package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot hand.
 */
public class FtcHand extends FtcSubSystemBase {
  private static final String TAG = "FtcHand";
  public static final String HAND_SERVO_NAME = "clawGrabServo";
  public static final String DELIVERY_SERVO_NAME = "clawDeliveryServo";
  public static final double HAND_CLOSE_POSITION = 0.5420;
  public static final double HAND_OPEN_POSITION = 0.4950;
  private static final double HAND_OPEN_TO_DELIVERY_POSITION = 0.5240;
  public static final double HAND_UP_POSITION = 0.5840;
  public static final double HAND_DOWN_POSITION = 0.4800;
  public static final long HAND_CLOSE_TIME_MS = 400;
  private static final long HAND_DELIVERY_TIME_MS = 300;
  public static final long HAND_OPEN_TIME_MS = HAND_CLOSE_TIME_MS;
  public static final long HAND_MOVE_DOWN_TIME_MS = 550;
  public static final long MOVE_MOVE_UP_TIME_MS = 750;

  private final boolean handEnabled = true;
  public boolean telemetryEnabled = true;
  private FtcBot parent;
  private Telemetry telemetry = null;
  public FtcServo handServo = null;
  public FtcServo deliveryServo = null;
  public FtcHandAsyncExecutor handAsyncExecutor = null;

  /* Constructor */
  public FtcHand() {
  }

  /**
   * Close the hand, thus grabbing the game element.
   *
   * @param waitTillClosed When true, waits till the hand closes.
   */
  public void close(boolean waitTillClosed) {
    FtcLogger.enter();
    if (handEnabled && handServo != null) {
      handServo.setPosition(HAND_CLOSE_POSITION);
      if (waitTillClosed) {
        FtcUtils.sleep(HAND_CLOSE_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry, FtcBot parent) {
    FtcLogger.enter();
    this.parent = parent;
    this.telemetry = telemetry;
    if (handEnabled) {
      handServo = new FtcServo(hardwareMap.get(Servo.class, HAND_SERVO_NAME));
      deliveryServo = new FtcServo(hardwareMap.get(Servo.class, DELIVERY_SERVO_NAME));
      if (handAsyncExecutor != null) {
        // Stop any previous, lingering executors.
        FtcLogger.error(TAG, "Found a previous instance of handAsyncExecutor");
        handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.Stop, false);
      }

      handAsyncExecutor = new FtcHandAsyncExecutor(this);
      Thread handAsyncExecutorThread = new Thread(handAsyncExecutor);
      handAsyncExecutorThread.start();

      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Rotates the hand down.
   *
   * @param waitTillClosed When true, waits till the hand is in down position.
   */
  public void rotateDown(boolean waitTillDown) {
    FtcLogger.enter();

    if (handEnabled && deliveryServo != null) {
      deliveryServo.setPosition(HAND_DOWN_POSITION);
      if (waitTillDown) {
        FtcUtils.sleep(HAND_MOVE_DOWN_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Rotates the hand up.
   *
   * @param waitTillClosed When true, waits till the hand is in up position.
   */
  public void rotateUp(boolean waitTillUp) {
    FtcLogger.enter();

    if (handEnabled && deliveryServo != null) {
      deliveryServo.setPosition(HAND_UP_POSITION);
      if (waitTillUp) {
        FtcUtils.sleep(MOVE_MOVE_UP_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Open the hand, thus releasing the game element.
   *
   * @param waitTillOpen When true, waits till the hand opens fully.
   */
  public void open(boolean waitTillOpen) {
    FtcLogger.enter();

    if (handEnabled && handServo != null) {
      handServo.setPosition(HAND_OPEN_POSITION);
      if (waitTillOpen) {
        FtcUtils.sleep(HAND_OPEN_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Open the hand slightly, thus releasing the game element.
   *
   * @param waitTillOpen When true, waits till the hand opens to release the game element.
   */
  public void openToDeliver(boolean waitTillDelivered) {
    FtcLogger.enter();
    if (handEnabled && handServo != null) {
      handServo.setPosition(HAND_OPEN_TO_DELIVERY_POSITION);
      if (waitTillDelivered) {
        FtcUtils.sleep(HAND_DELIVERY_TIME_MS);
      }
    }

    FtcLogger.exit();
  }

  /**
   * Operates the hand using the gamePads.
   * Right bumper -> open the hand.
   * Right trigger -> close the hand.
   *
   * @param gamePad1 The first gamePad to use.
   * @param gamePad2 The second gamePad to use.
   */
  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    if (handEnabled) {
      if (gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
        if (handAsyncExecutor.handIsDown()) {
          if (parent == null) {
            // No parent, we are likely testing Arm module.
            if (handAsyncExecutor.handIsOpen()) {
              handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.Close, false);
            } else if (handAsyncExecutor.handIsClosed()) {
              handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.RotateUp, false);
            } else {
              FtcLogger.error(TAG, "Hand is down and neither open or closed");
            }
          } else {
            if (handAsyncExecutor.handIsOpen()) {
              handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.Close, false);
            } else if (handAsyncExecutor.handIsClosed()) {
              // Hand delivers if lift is in low position, arm in receive position, hand is closed
              handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.RotateUp, false);
            } else {
              FtcLogger.error(TAG, "Hand is down and neither open or closed");
            }
          }
        } else if (handAsyncExecutor.handIsUp()) {
          handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.OpenToDeliver, false);
        } else {
          FtcLogger.error(TAG, "Hand is neither up nor down");
        }
      } else if (gamePad1.leftBumperWasPressed() || gamePad2.leftBumperWasPressed()) {
        if (handAsyncExecutor.handIsUp()) {
          handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.RotateDown, false);
        } else if (handAsyncExecutor.handIsDown()) {
          if (handAsyncExecutor.handIsClosed()) {
            handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.Open, false);
          } else if (handAsyncExecutor.handIsOpen()) {
            FtcLogger.debug(TAG, "Bumper no op: hand closed, hand is down and open");
          } else {
            FtcLogger.error(TAG, "Hand is down and neither open or closed");
          }
        } else {
          FtcLogger.error(TAG, "hand neither up nor down!");
        }
      }
    }
  }

  /**
   * Displays hand servo telemetry. Helps with debugging.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (handEnabled && telemetryEnabled && handServo != null && deliveryServo != null) {
      telemetry.addData(TAG, String.format(Locale.US, "Hand %5.4f, delivery %5.4f",
          handServo.getPosition(), deliveryServo.getPosition()));
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    if (handEnabled) {
      handServo.getController().pwmEnable();
      deliveryServo.getController().pwmEnable();
      rotateDown(false);
      open(false);
    }

    FtcLogger.exit();
  }

  /**
   * Stops the hand.
   */
  public void stop() {
    FtcLogger.enter();
    if (handEnabled) {
      handAsyncExecutor.setOperation(FtcHandAsyncExecutor.HandOperation.Stop, false);
      handAsyncExecutor = null;
      handServo.getController().pwmDisable();
      deliveryServo.getController().pwmDisable();
    }

    FtcLogger.exit();
  }
}
