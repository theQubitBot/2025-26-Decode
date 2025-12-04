package org.firstinspires.ftc.teamcode.qubit.core.TrollBots;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBulkRead;
import org.firstinspires.ftc.teamcode.qubit.core.FtcDriveTrain;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.MatchConfig;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

public class BotC extends BaseBot {
  public BotC() {
    TAG = "BotC";
  }

  public void disableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = false;
    driveTrain.telemetryEnabled = false;
    FtcLogger.exit();
  }

  public void enableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = true;
    driveTrain.telemetryEnabled = true;
    FtcLogger.exit();
  }

  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;

    bulkRead = new FtcBulkRead();
    bulkRead.init(hardwareMap, telemetry, autoOp);

    config = new MatchConfig();
    config.init(hardwareMap, telemetry, autoOp);

    driveTrain = new FtcDriveTrain(this);
    driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
    driveTrain.init(hardwareMap, telemetry, autoOp);

    telemetry.addData(TAG, "initialized");
    FtcLogger.exit();
  }

  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();

    bulkRead.clearBulkCache();
    driveTrain.operate(gamePad1, gamePad2, loopTime, runtime);

    driveTrain.showTelemetry();
    showGamePadTelemetry(gamePad1);

    FtcLogger.exit();
  }

  public void start() {
    FtcLogger.enter();
    FtcLogger.exit();
  }

  public void stop() {
    FtcLogger.enter();
    driveTrain.stop();

    FtcLogger.exit();
  }
}
