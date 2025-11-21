package org.firstinspires.ftc.teamcode.qubit.core;

/**
 * A class to asynchronously operate the sorter.
 */
public class FtcSorterAsyncUpdater implements Runnable {
  public static final String TAG = "FtcSorterAsyncUpdater";
  private final FtcSorter ftcSorter;
  private boolean stopRequested;

  /**
   * Constructor.
   *
   * @param ftcSorter The actual Sorter.
   */
  public FtcSorterAsyncUpdater(FtcSorter ftcSorter) {
    FtcLogger.enter();
    this.ftcSorter = ftcSorter;
    stopRequested = false;
    FtcLogger.exit();
  }

  private synchronized boolean keepRunning() {
    return !stopRequested;
  }

  /**
   * Requests a non-blocking sorter stop.
   */
  public synchronized void stop() {
    stopRequested = true;
  }

  /**
   * Operates the sorter in a tight loop.
   */
  public void run() {
    while (keepRunning()) {
      ftcSorter.operate();
      FtcUtils.sleep(1);
    }

    FtcLogger.info(TAG, "Async thread stopped.");
  }
}
