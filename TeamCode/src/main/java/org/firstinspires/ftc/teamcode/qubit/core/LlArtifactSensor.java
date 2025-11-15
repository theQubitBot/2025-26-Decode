package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * A camera based artifact sensor.
 */
public class LlArtifactSensor {
  public boolean telemetryEnabled = true;
  public static final String LIMELIGHT_NAME = "limelight";
  private Telemetry telemetry;
  private Limelight3A limelight = null;

  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;
    limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
    limelight.pipelineSwitch(0);
  }

  public boolean isGreenVisible() {
    boolean greenVisible = false;
    LLResult result = limelight.getLatestResult();
    if (result.isValid()) {
      List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
      greenVisible = colorResults != null && !colorResults.isEmpty();
    }

    return greenVisible;
  }

  /**
   * Function to add telemetry about Artifact detection.
   */
  @SuppressLint("DefaultLocale")
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled) {
      telemetry.addData("Artifact color", isGreenVisible() ? "green" : "purple");
    }

    FtcLogger.exit();
  }

  /**
   * Code to run ONCE when the driver hits PLAY
   */
  public void start() {
    FtcLogger.enter();
    limelight.start();
    FtcLogger.exit();
  }

  /**
   * Stops artifact detection processing.
   */
  public void stop() {
    FtcLogger.enter();
    limelight.stop();
    FtcLogger.exit();
  }
}
