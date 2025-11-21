package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.ObeliskTagEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

@Autonomous(group = "Official", preselectTeleOp = "DriverTeleOp")
public class AutoOp extends LinearOpMode {
  private ElapsedTime runtime = null;
  FtcBot robot = null;
  Follower follower;
  OptionBase optionBase;
  OptionBlueGoal optionBlueGoal;
  OptionBlueAudience optionBlueAudience;
  OptionRedGoal optionRedGoal;
  OptionRedAudience optionRedAudience;

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
    // Enable and reset servos
    robot.start();
    robot.cannon.stop();
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
      if (robot.config.robotPosition == RobotPositionEnum.GOAL) {
        optionBlueGoal = new OptionBlueGoal(this, robot, follower).init();
        optionBase = optionBlueGoal;
      } else if (robot.config.robotPosition == RobotPositionEnum.AUDIENCE) {
        optionBlueAudience = new OptionBlueAudience(this, robot, follower).init();
        optionBase = optionBlueAudience;
      } else {
        throw new IllegalStateException(String.format("Unsupported robot position: %s",
            robot.config.robotPosition.toString()));
      }
    } else if (robot.config.allianceColor == AllianceColorEnum.RED) {
      if (robot.config.robotPosition == RobotPositionEnum.GOAL) {
        optionRedGoal = new OptionRedGoal(this, robot, follower).init();
        optionBase = optionRedGoal;
      } else if (robot.config.robotPosition == RobotPositionEnum.AUDIENCE) {
        optionRedAudience = new OptionRedAudience(this, robot, follower).init();
        optionBase = optionRedAudience;
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
      if (robot.config.robotPosition == RobotPositionEnum.AUDIENCE) {
        ObeliskTagEnum ote = robot.aprilTag.getObeliskTag();
        if (ote != ObeliskTagEnum.UNKNOWN) {
          robot.config.obeliskTagEnum = ote;
          robot.config.saveToFile();
        }
      }

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
        FtcUtils.interruptibleSleep(1000, this);
        countDown--;
      }
    }

    if (!opModeIsActive()) return;
    telemetry.addData(FtcUtils.TAG, "Auto Op started.");
    telemetry.update();
    if (!opModeIsActive()) return;
    robot.sorter.startAsyncUpdater();

    if (robot.config.allianceColor == AllianceColorEnum.BLUE) {
      if (robot.config.robotPosition == RobotPositionEnum.GOAL) {
        optionBlueGoal.execute();
      } else {
        optionBlueAudience.execute();
      }
    } else {
      if (robot.config.robotPosition == RobotPositionEnum.GOAL) {
        optionRedGoal.execute();
      } else {
        optionRedAudience.execute();
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
      autoOpExecutionDuration = Math.round(runtime.seconds());
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
