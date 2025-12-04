package org.firstinspires.ftc.teamcode.qubit.core;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * A class to asynchronously operate the sorter.
 */
public class FtcSorterAsyncUpdater implements Runnable {
  public static final String TAG = "FtcSorterAsyncUpdater";
  private final FtcSorter ftcSorter;
  private final AtomicBoolean stopRequested;

  /**
   * Constructor.
   *
   * @param ftcSorter The actual Sorter.
   */
  public FtcSorterAsyncUpdater(FtcSorter ftcSorter) {
    FtcLogger.enter();
    this.ftcSorter = ftcSorter;
    stopRequested = new AtomicBoolean(false);
    FtcLogger.exit();
  }

  private boolean keepRunning() {
    return !stopRequested.get();
  }

  /**
   * Requests a non-blocking sorter stop.
   */
  public void stop() {
    stopRequested.set(true);
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
