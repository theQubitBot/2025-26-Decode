package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TrollBotEnum;

/**
 * A class to manage the built-in IMU.
 */
public class FtcBhi260apImu extends FtcSubSystemBase {
  private static final String TAG = "FtcBhi260apImu";
  public boolean telemetryEnabled = true;
  Telemetry telemetry = null;
  private final FtcBot parent;
  private IMU imu = null;
  private boolean imuIsGood = true;
  private static final Object directionLock = new Object();
  private double heading = 0.0; // Increases when robot turns left
  private double pitch = 0.0;
  private double roll = 0.0;

  /* Constructor */
  public FtcBhi260apImu(FtcBot robot) {
    parent = robot;
  }

  /**
   * A thread safe method to get IMU heading.
   *
   * @return The IMU heading.
   */
  public double getHeading() {
    double currentHeading;
    synchronized (directionLock) {
      currentHeading = heading;
    }

    return currentHeading;
  }

  /**
   * A thread safe method to get IMU pitch.
   *
   * @return The IMU pitch.
   */
  public double getPitch() {
    double currentPitch;
    synchronized (directionLock) {
      currentPitch = pitch;
    }

    return currentPitch;
  }

  /**
   * A thread safe method to get IMU roll.
   *
   * @return The IMU roll.
   */
  public double getRoll() {
    double currentRoll;
    synchronized (directionLock) {
      currentRoll = roll;
    }

    return currentRoll;
  }

  public boolean imuIsGood() {
    boolean isGood;
    synchronized (directionLock) {
      isGood = imuIsGood;
    }

    if (!isGood) {
      telemetry.addData(TAG, "IMU is in a bad state.");
    }

    return isGood;
  }

  /**
   * Initialize IMU. Robot must be still during IMU initialization (calibration).
   * See <a href="https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html">...</a>
   * for initialization times.
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry object to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    imu = hardwareMap.get(IMU.class, "imu");
    if (imu != null) {
      RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
      RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;

      if (parent.trollBot == TrollBotEnum.TrollBotA) {
        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
      } else if (parent.trollBot == TrollBotEnum.TrollBotB) {
        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
      } else if (parent.trollBot == TrollBotEnum.TrollBotC) {
        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
      } else if (parent.trollBot == TrollBotEnum.TrollBotD) {
        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
      } else {
        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
      }

      RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
          logoFacingDirection, usbFacingDirection);
      imu.initialize(new IMU.Parameters(orientationOnRobot));
      imu.resetYaw();

      // Get a reading before the async reader is started.
      read();
      showTelemetry();
      telemetry.addData(TAG, "initialized");
    } else {
      telemetry.addData(TAG, "not initialized");
    }

    FtcLogger.exit();
  }

  /**
   * Read the IMU and store information for later use.
   */
  public void read() {
    FtcLogger.enter();
    if (imu != null) {
      YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
      synchronized (directionLock) {
        heading = yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
        pitch = yawPitchRollAngles.getPitch(AngleUnit.DEGREES);
        roll = yawPitchRollAngles.getRoll(AngleUnit.DEGREES);
        imuIsGood = yawPitchRollAngles.getAcquisitionTime() != 0;
      }
    }

    FtcLogger.exit();
  }

  /**
   * Display IMU (gyro) telemetry.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled) {
      telemetry.addData("Heading/Yaw", "%.2f", getHeading());
      telemetry.addData("Pitch", "%.2f", getPitch());
      telemetry.addData("Roll", "%.2f", getRoll());
    }

    FtcLogger.exit();
  }
}
