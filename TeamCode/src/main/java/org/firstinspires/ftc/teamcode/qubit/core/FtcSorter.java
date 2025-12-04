package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

/**
 * A class to manage the artifact sorter.
 */
public class FtcSorter extends FtcSubSystemBase {
  private static final String TAG = "FtcSorter";
  public static final String SORTER_SERVO_NAME = "sorterServo";
  public static final double SORTER_GREEN_POSITION = 0.4910;
  public static final double SORTER_PURPLE_POSITION = 0.4320;
  public static final double SORTER_STRAIGHT_POSITION =
      (SORTER_GREEN_POSITION + SORTER_PURPLE_POSITION) / 2;
  public static final double SORTER_GREEN_PUSH_POSITION = 0.3960;
  public static final double SORTER_PURPLE_PUSH_POSITION = 0.5290;
  public static final int SORTER_SERVO_MOVE_TIME = 200; // milliseconds
  private final boolean sorterEnabled = true;
  public boolean telemetryEnabled = true;
  private Telemetry telemetry = null;
  public FtcServo sorterServo = null;
  private FtcSorterAsyncUpdater asyncUpdater = null;
  private Thread asyncUpdaterThread = null;
  private final BaseBot parent;

  /* Constructor */
  public FtcSorter(BaseBot robot) {
    parent = robot;
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    if (sorterEnabled) {
      sorterServo = new FtcServo(hardwareMap.get(Servo.class, SORTER_SERVO_NAME));
      sorterServo.setDirection(Servo.Direction.FORWARD);

      if (autoOp) {
        stopAsyncOperations();
        startAsyncOperations();
      }

      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not enabled");
    }

    FtcLogger.exit();
  }

  /**
   * Operates the sorter automatically using artifact sensor.
   * Used primarily by autoOp via asyncUpdater.
   */
  public void operate() {
    if (parent != null && parent.artifactSensor != null) {
      LlArtifactSensor.ArtifactColor artifactColor = parent.artifactSensor.getArtifactColor();
      if (artifactColor == LlArtifactSensor.ArtifactColor.GREEN) {
        setGreen(false);
      } else if (artifactColor == LlArtifactSensor.ArtifactColor.PURPLE) {
        setPurple(false);
      } // else - don't move if no artifact is detected.
    }
  }

  /**
   * Operates the sorter automatically using artifact sensor OR
   * using gamepads.
   *
   * @param gamePad1 Gamepad1 to use.
   * @param gamePad2 Gamepad2 to use.
   */
  @Override
  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();
    if (gamePad1.a || gamePad2.a) {
      setStraight(false);
    } else {
      operate();
    }

    FtcLogger.exit();
  }

  /**
   * Makes the sorter push green artifacts onto trigger..
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void pushGreen(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_GREEN_PUSH_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Makes the sorter push green artifacts onto trigger..
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void pushPurple(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_PURPLE_PUSH_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Sets the sorter to the green barrel.
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void setGreen(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_GREEN_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Sets the sorter to the purple barrel.
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void setPurple(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_PURPLE_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Sets the sorter straight ahead.
   *
   * @param waitTillCompletion When True, waits for sorter to complete the movement.
   */
  public void setStraight(boolean waitTillCompletion) {
    if (sorterEnabled && sorterServo != null) {
      sorterServo.setPosition(SORTER_STRAIGHT_POSITION);
      if (waitTillCompletion) {
        FtcUtils.sleep(SORTER_SERVO_MOVE_TIME);
      }
    }
  }

  /**
   * Displays sorter telemetry. Helps with debugging.
   */
  @Override
  public void showTelemetry() {
    FtcLogger.enter();
    if (sorterEnabled && telemetryEnabled && telemetry != null) {
      double position = sorterServo.getPosition();
      telemetry.addData(TAG, "%5.4f (%s)",
          position,
          FtcUtils.areEqual(position, SORTER_GREEN_POSITION, FtcUtils.EPSILON4) ? "Green" :
              FtcUtils.areEqual(position, SORTER_PURPLE_POSITION, FtcUtils.EPSILON4) ? "Purple" :
                  FtcUtils.areEqual(position, SORTER_STRAIGHT_POSITION, FtcUtils.EPSILON4) ? "Straight" : "Unknown");
    }

    FtcLogger.exit();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    if (sorterEnabled && sorterServo != null) {
      if (sorterServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
        sorterServo.getController().pwmEnable();
      }

      // Move the servo for correct initialization.
      setGreen(false);
      FtcUtils.sleep(1);
      setPurple(false);
    }

    FtcLogger.exit();
  }

  private void startAsyncOperations() {
    asyncUpdater = new FtcSorterAsyncUpdater(this);
    asyncUpdaterThread = new Thread(asyncUpdater, FtcSorterAsyncUpdater.TAG);
    asyncUpdaterThread.setPriority(Thread.NORM_PRIORITY + 1); // Priority 6 for sorter updates
    asyncUpdaterThread.setDaemon(true); // Auto-terminate on shutdown for fast OpMode transitions
    asyncUpdaterThread.start();
  }

  /**
   * Stops the sorter.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    stopAsyncOperations();
    FtcLogger.exit();
  }

  private void stopAsyncOperations() {
    // Stop and join existing thread before creating a new one
    if (asyncUpdater != null) {
      asyncUpdater.stop();

      // Wait for thread to finish before cleaning up resources
      if (asyncUpdaterThread != null && asyncUpdaterThread.isAlive()) {
        try {
          asyncUpdaterThread.join(10);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }

      asyncUpdater = null;
      asyncUpdaterThread = null;
    }
  }
}
