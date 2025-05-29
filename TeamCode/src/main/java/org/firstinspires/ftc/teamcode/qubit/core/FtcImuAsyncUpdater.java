package org.firstinspires.ftc.teamcode.qubit.core;

/**
 * A class to asynchronously read the IMU values.
 */
public class FtcImuAsyncUpdater implements Runnable {
  public static final String TAG = "FtcImuAsyncUpdater";
  private final FtcImu imu;
  private boolean stopRequested;

  /**
   * Constructor.
   *
   * @param imu The actual IMU.
   */
  public FtcImuAsyncUpdater(FtcImu imu) {
    FtcLogger.enter();
    this.imu = imu;
    stopRequested = false;
    FtcLogger.exit();
  }

  private synchronized boolean keepRunning() {
    return !stopRequested;
  }

  /**
   * Requests an IMU stop, ASAP. Call is non-blocking.
   */
  public synchronized void stop() {
    stopRequested = true;
  }

  /**
   * Reads the IMU in a tight loop.
   */
  public void run() {
    while (keepRunning()) {
      imu.readAsync();
      FtcUtils.sleep(1);
    }

    FtcLogger.info(TAG, "Async thread stopped.");
  }
}
