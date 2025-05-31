package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A class to manage the sensors.
 */
public class FtcSensors extends FtcSubSystemBase {
  private static final String TAG = "FtcSensors";
  private Telemetry telemetry = null;
  public boolean showTelemetry = true;

  public FtcSensors() {
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  public void init(HardwareMap hardwareMap, Telemetry telemetry, FtcBot robot) {
    FtcLogger.enter();
    // Save reference to Hardware map
    this.telemetry = telemetry;
    showTelemetry();
    FtcLogger.exit();
  }

  public void showTelemetry() {
    FtcLogger.enter();
    FtcLogger.exit();
  }
}
