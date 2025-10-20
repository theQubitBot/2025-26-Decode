package org.firstinspires.ftc.teamcode.qubit.autoOps;
// Joshua wrote this

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

@Autonomous(group = "Official", preselectTeleOp = "DriverTeleOp")
public class AutoOp extends LinearOpMode {
  private ElapsedTime runtime = null;
  FtcBot robot = null;
  Follower follower;
  OptionBase optionBase;
  OptionBlueLargeTriangle optionBlueLargeTriangle;
  OptionBlueSmallTriangle optionBlueSmallTriangle;
  OptionRedLargeTriangle optionRedLargeTriangle;
  OptionRedSmallTriangle optionRedSmallTriangle;

  @Override
  public void runOpMode() {
    FtcLogger.enter();
    initializeModules();
    processStuffDuringInit();
    executeAutonomousOperation();
    waitForEnd();
    FtcLogger.exit();
  }

  /**
   * Initialize the variables, etc.
   */
  private void initializeModules() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing. Please wait...");
    telemetry.update();

    // Initialize robot.
    robot = new FtcBot();
    robot.init(hardwareMap, telemetry, true);
    robot.blinkinLed.set(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    robot.aprilTag.pointAtObelisk();

    if (FtcUtils.DEBUG) {
      robot.enableTelemetry();
    } else {
      // Disable telemetry to speed things up.
      robot.disableTelemetry();
    }

    // Initialize Pedro for robot paths and trajectories
    // Must initialize this after robot.driveTrain initialization since driveTrain
    // sets the motors to run without encoders.
    follower = Constants.createFollower(hardwareMap);
    if (robot.config.allianceColor == AllianceColorEnum.BLUE) {
      if (robot.config.robotPosition == RobotPositionEnum.LARGE_TRIANGLE) {
        optionBlueLargeTriangle = new OptionBlueLargeTriangle(this, robot, follower).init();
        optionBase = optionBlueLargeTriangle;
      } else if (robot.config.robotPosition == RobotPositionEnum.SMALL_TRIANGLE) {
        optionBlueSmallTriangle = new OptionBlueSmallTriangle(this, robot, follower).init();
        optionBase = optionBlueSmallTriangle;
      } else {
        throw new IllegalStateException(String.format("Unsupported robot position: %s",
            robot.config.robotPosition.toString()));
      }
    } else if (robot.config.allianceColor == AllianceColorEnum.RED) {
      if (robot.config.robotPosition == RobotPositionEnum.LARGE_TRIANGLE) {
        optionRedLargeTriangle = new OptionRedLargeTriangle(this, robot, follower).init();
        optionBase = optionRedLargeTriangle;
      } else if (robot.config.robotPosition == RobotPositionEnum.SMALL_TRIANGLE) {
        optionRedSmallTriangle = new OptionRedSmallTriangle(this, robot, follower).init();
        optionBase = optionRedSmallTriangle;
      } else {
        throw new IllegalStateException(String.format("Unsupported robot position: %s",
            robot.config.robotPosition.toString()));
      }
    } else {
      throw new IllegalStateException(String.format("Unsupported alliance color: %s",
          robot.config.allianceColor.toString()));
    }

    FtcLogger.exit();
  }

  private void processStuffDuringInit() {
    FtcLogger.enter();

    while (opModeInInit()) {
      robot.config.showConfiguration();
      telemetry.addLine();

      telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play.");
      telemetry.update();
      FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    FtcLogger.exit();
  }

  /**
   * Invokes autonomous operation based on match configuration.
   */
  private void executeAutonomousOperation() {
    FtcLogger.enter();
    runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    if (robot.config.delayInSeconds > 0) {
      long countDown = robot.config.delayInSeconds;
      while (countDown > 0) {
        if (!opModeIsActive()) return;
        telemetry.addData(FtcUtils.TAG, "Delaying start by %d seconds",
            robot.config.delayInSeconds);
        telemetry.addData(FtcUtils.TAG, "Countdown %d seconds", countDown);
        telemetry.update();
        FtcUtils.sleep(1000);
        countDown--;
      }
    }

    if (!opModeIsActive()) return;
    telemetry.addData(FtcUtils.TAG, "Auto Op started.");
    telemetry.update();
    if (!opModeIsActive()) return;

    // Enable and reset servos
    robot.start();

    if (robot.config.allianceColor == AllianceColorEnum.BLUE) {
      if (robot.config.robotPosition == RobotPositionEnum.LARGE_TRIANGLE) {
        optionBlueLargeTriangle.execute();
      } else {
        optionBlueSmallTriangle.execute();
      }
    } else {
      if (robot.config.robotPosition == RobotPositionEnum.LARGE_TRIANGLE) {
        optionBlueLargeTriangle.execute();
      } else {
        optionBlueLargeTriangle.execute();
      }
    }

    robot.stop();
    FtcLogger.exit();
  }

  /**
   * Waits for the autonomous operation to end.
   * If using FOD, saves gyro Heading for use by TeleOp.
   */
  private void waitForEnd() {
    FtcLogger.enter();

    double autoOpExecutionDuration = 0;
    if (runtime != null) {
      autoOpExecutionDuration = runtime.seconds();
    }

    while (optionBase != null && optionBase.saveAndTest()) {
      telemetry.addData(FtcUtils.TAG, "Auto Op took %.0f seconds.", autoOpExecutionDuration);
      telemetry.addData(FtcUtils.TAG, "Waiting for auto Op to end.");
      telemetry.update();
      FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    FtcLogger.exit();
  }
}
