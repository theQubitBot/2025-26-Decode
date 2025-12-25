package org.firstinspires.ftc.teamcode.qubit.core.TrollBots;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.FtcAprilTag;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBlinkinLed;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBulkRead;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcDriveTrain;
import org.firstinspires.ftc.teamcode.qubit.core.FtcIntake;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcSorter;
import org.firstinspires.ftc.teamcode.qubit.core.LlArtifactSensor;
import org.firstinspires.ftc.teamcode.qubit.core.MatchConfig;
import org.firstinspires.ftc.teamcode.qubit.core.TeleOpLocalizer;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

public class BotA extends BaseBot {
  public BotA() {
    TAG = "BotA";
  }

  public void disableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = false;
    aprilTag.telemetryEnabled = false;
    artifactSensor.telemetryEnabled = false;
    blinkinLed.telemetryEnabled = false;
    cannon.telemetryEnabled = false;
    driveTrain.telemetryEnabled = false;
    intake.telemetryEnabled = false;
    localizer.telemetryEnabled = false;
    sorter.telemetryEnabled = false;
    FtcLogger.exit();
  }

  public void enableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = true;
    aprilTag.telemetryEnabled = true;
    artifactSensor.telemetryEnabled = true;
    blinkinLed.telemetryEnabled = true;
    cannon.telemetryEnabled = true;
    driveTrain.telemetryEnabled = true;
    intake.telemetryEnabled = true;
    localizer.telemetryEnabled = true;
    sorter.telemetryEnabled = true;
    FtcLogger.exit();
  }

  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;

    // Initialize config first since it is used by other sub systems' init.
    config = new MatchConfig();
    config.init(hardwareMap, telemetry, autoOp);

    aprilTag = new FtcAprilTag(this);
    aprilTag.init(hardwareMap, telemetry, autoOp);

    artifactSensor = new LlArtifactSensor();
    artifactSensor.init(hardwareMap, telemetry, autoOp);

    blinkinLed = new FtcBlinkinLed(this);
    blinkinLed.init(hardwareMap, telemetry, autoOp);

    bulkRead = new FtcBulkRead();
    bulkRead.init(hardwareMap, telemetry, autoOp);

    cannon = new FtcCannon(this);
    cannon.init(hardwareMap, telemetry, autoOp);

    localizer = new TeleOpLocalizer(this);
    localizer.init(hardwareMap, telemetry, autoOp);

    driveTrain = new FtcDriveTrain(this);
    driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
    driveTrain.init(hardwareMap, telemetry, autoOp);

    intake = new FtcIntake(this);
    intake.init(hardwareMap, telemetry, autoOp);

    sorter = new FtcSorter(this);
    sorter.init(hardwareMap, telemetry, autoOp);

    telemetry.addData(TAG, "initialized");
    FtcLogger.exit();
  }

  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();

    bulkRead.clearBulkCache();
    blinkinLed.operate(gamePad1, gamePad2, loopTime, runtime);
    cannon.operate(gamePad1, gamePad2, loopTime, runtime);
    driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);
    intake.operate(gamePad1, gamePad2, loopTime, runtime);
    sorter.operate(gamePad1, gamePad2, loopTime, runtime);
    localizer.operate(gamePad1, gamePad2, loopTime, runtime);

    aprilTag.showTelemetry();
    blinkinLed.showTelemetry();
    cannon.showTelemetry();
    driveTrain.showTelemetry();
    intake.showTelemetry();
    sorter.showTelemetry();
    localizer.showTelemetry();
    showGamePadTelemetry(gamePad1);

    FtcLogger.exit();
  }

  public void start() {
    FtcLogger.enter();
    aprilTag.start();
    artifactSensor.start();
    blinkinLed.start();
    cannon.start();
    intake.start();
    sorter.start();
    localizer.start();
    FtcLogger.exit();
  }

  public void stop() {
    FtcLogger.enter();
    aprilTag.stop();
    artifactSensor.stop();
    blinkinLed.stop();
    cannon.stop();
    driveTrain.stop();
    intake.stop();
    sorter.stop();
    localizer.stop();
    FtcLogger.exit();
  }
}
