package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A camera based artifact sensor.
 */
public class LlArtifactSensor extends FtcSubSystemBase {
  public boolean telemetryEnabled = true;
  public static final String LIMELIGHT_NAME = "limelight";
  public static final double HSV_GREEN_LOW = 55.0;
  public static final double HSV_PURPLE_LOW = 115.0;
  private Telemetry telemetry;
  private Limelight3A limelight = null;

  public enum ArtifactColor {
    GREEN,
    PURPLE,
    UNKNOWN
  }

  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    this.telemetry = telemetry;
    limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
    limelight.pipelineSwitch(2);
  }

  public ArtifactColor getArtifactColor() {
    ArtifactColor artifactColor = ArtifactColor.UNKNOWN;
    if (limelight != null) {
      double[] llPython = limelight.getLatestResult().getPythonOutput();
      if (llPython != null && llPython.length > 0) {
        if (FtcUtils.areEqual(llPython[0], HSV_GREEN_LOW, FtcUtils.EPSILON1)) {
          artifactColor = ArtifactColor.GREEN;
        } else if (FtcUtils.areEqual(llPython[0], HSV_PURPLE_LOW, FtcUtils.EPSILON1)) {
          artifactColor = ArtifactColor.PURPLE;
        }
      }
    }

    return artifactColor;
  }

  /**
   * Function to add telemetry about Artifact detection.
   */
  @Override
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
  @Override
  public void start() {
    FtcLogger.enter();
    limelight.start();
    FtcLogger.exit();
  }

  /**
   * Stops artifact detection processing.
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    limelight.stop();
    FtcLogger.exit();
  }
}
