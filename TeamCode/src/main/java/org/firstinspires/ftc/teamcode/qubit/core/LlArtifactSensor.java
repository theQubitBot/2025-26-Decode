package org.firstinspires.ftc.teamcode.qubit.core;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A camera based artifact sensor.
 */
public class LlArtifactSensor {
  public boolean telemetryEnabled = true;
  public static final String LIMELIGHT_NAME = "limelight";
  private Telemetry telemetry;
  private Limelight3A limelight = null;

  public enum ArtifactColor {
    GREEN,
    PURPLE,
    UNKNOWN
  }

  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;
    limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
    limelight.pipelineSwitch(2);
  }

  public ArtifactColor getArtifactColor() {
    ArtifactColor artifactColor = ArtifactColor.UNKNOWN;
    LLResult result = limelight.getLatestResult();
    double[] llPython = result.getPythonOutput();
    if (llPython != null) {
      if (llPython.length > 0) {
        if (llPython[0] == 55) {
          artifactColor = ArtifactColor.GREEN;
        } else if (llPython[0] == 145) {
          artifactColor = ArtifactColor.PURPLE;
        }
      } else {
        telemetry.addLine("llPython.length is 0");
      }
    } else {
      telemetry.addLine("llPython is null");
    }

    return artifactColor;
  }

  /**
   * Function to add telemetry about Artifact detection.
   */
  @SuppressLint("DefaultLocale")
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled) {
      telemetry.addData("Artifact color", getArtifactColor());
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
