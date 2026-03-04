package org.firstinspires.ftc.teamcode.qubit.core;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * A class to asynchronously operate the localizer.
 */
public class TeleOpLocalizerAsyncUpdater implements Runnable {
  public static final String TAG = "TeleOpLocalizerAsyncUpdater";
  public static final int UPDATE_INTERVAL = 5; // milliseconds
  private final TeleOpLocalizer teleOpLocalizer;
  private final AtomicBoolean stopRequested;

  /**
   * Constructor.
   *
   * @param teleOpLocalizer The actual localizer.
   */
  public TeleOpLocalizerAsyncUpdater(TeleOpLocalizer teleOpLocalizer) {
    this.teleOpLocalizer = teleOpLocalizer;
    stopRequested = new AtomicBoolean(false);
  }

  private boolean keepRunning() {
    return !stopRequested.get();
  }

  /**
   * Requests a non-blocking localizer stop.
   */
  public void stop() {
    stopRequested.set(true);
  }

  /**
   * Operates the localizer in a tight loop.
   */
  public void run() {
    while (keepRunning()) {
      teleOpLocalizer.savePosition();
      FtcUtils.sleep(UPDATE_INTERVAL);
    }
  }
}
