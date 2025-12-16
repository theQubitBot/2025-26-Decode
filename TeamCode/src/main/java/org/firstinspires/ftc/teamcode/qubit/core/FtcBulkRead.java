package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * A class to manage the bulk read operations.
 */
public class FtcBulkRead extends FtcSubSystemBase {
  private static final String TAG = "FtcBulkRead";
  private List<LynxModule> allLynxModules = null;
  private LynxModule.BulkCachingMode cachingMode;
  private Telemetry telemetry = null;

  /**
   * Constructor.
   */
  public FtcBulkRead() {
    // Start off with the default OFF mode.
    cachingMode = LynxModule.BulkCachingMode.OFF;
  }

  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   */
  @Override
  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    FtcLogger.enter();
    this.telemetry = telemetry;

    // Get a list of Control/Expansion Hub modules to manage caching support.
    allLynxModules = hardwareMap.getAll(LynxModule.class);
    if (autoOp) {
      setCachingMode(LynxModule.BulkCachingMode.AUTO);
    } else {
      setCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    FtcLogger.exit();
  }

  /**
   * Gets the current bulk caching mode.
   *
   * @return The current bulk caching mode.
   */
  public LynxModule.BulkCachingMode getCachingMode() {
    return cachingMode;
  }

  /**
   * Sets the bulk caching mode. This operation is a no-op if
   * the new caching mode is same as the current caching mode.
   */
  public void setCachingMode(LynxModule.BulkCachingMode newCachingMode) {
    FtcLogger.enter();
    if (allLynxModules != null && !allLynxModules.isEmpty()) {
      for (LynxModule lynxModule : allLynxModules) {
        lynxModule.setBulkCachingMode(newCachingMode);
      }

      cachingMode = newCachingMode;
    }

    FtcLogger.exit();
  }

  /**
   * Display bulk caching mode.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetry != null) {
      telemetry.addData(TAG, "BulkCachingMode %s", cachingMode);
    }

    FtcLogger.exit();
  }

  /**
   * Clears the bulk cache.
   */
  public void clearBulkCache() {
    FtcLogger.enter();
    if (cachingMode == LynxModule.BulkCachingMode.MANUAL &&
        allLynxModules != null && !allLynxModules.isEmpty()) {
      for (LynxModule lynxModule : allLynxModules) {
        lynxModule.clearBulkCache();
      }
    }

    FtcLogger.exit();
  }
}
