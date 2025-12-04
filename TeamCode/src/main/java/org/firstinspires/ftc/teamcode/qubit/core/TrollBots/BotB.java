package org.firstinspires.ftc.teamcode.qubit.core.TrollBots;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;

public class BotB extends BaseBot {
  public BotB() {
    TAG = "BotB";
  }

  public void disableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = false;
    FtcLogger.exit();
  }

  public void enableTelemetry() {
    FtcLogger.enter();
    telemetryEnabled = true;
    FtcLogger.exit();
  }

  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;

    telemetry.addData(TAG, "initialized");
    FtcLogger.exit();
  }

  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    FtcLogger.enter();
    bulkRead.clearBulkCache();
    showGamePadTelemetry(gamePad1);
    FtcLogger.exit();
  }

  public void start() {
    FtcLogger.enter();
    FtcLogger.exit();
  }

  public void stop() {
    FtcLogger.enter();
    FtcLogger.exit();
  }
}
