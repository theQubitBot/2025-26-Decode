package org.firstinspires.ftc.teamcode.qubit.core;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * A class to asynchronously execute the processor.
 */
public class FtcAprilTagAsyncUpdater implements Runnable {
  public static final String TAG = "FtcAprilTagAsyncUpdater";
  private static final long MAX_UPDATE_INTERVAL = 10; // milliseconds
  private final AprilTagProcessor processor;
  private final AtomicBoolean stopRequested;
  private final ArrayList<AprilTagDetection> detections;
  private final List<AprilTagDetection> detectionsCopy;
  private final AtomicLong lastCallTime, lastCallInterval; // milliseconds
  private final Object tagLock;

  public FtcAprilTagAsyncUpdater(AprilTagProcessor processor) {
    FtcLogger.enter();
    this.processor = processor;
    stopRequested = new AtomicBoolean(false);
    detections = new ArrayList<>(5);

    // There are 5 tags on the field
    detectionsCopy = new ArrayList<>(5);
    lastCallTime = new AtomicLong(System.currentTimeMillis());
    lastCallInterval = new AtomicLong(MAX_UPDATE_INTERVAL);
    tagLock = new Object();
    FtcLogger.exit();
  }

  /**
   * Returns a copy of current detections.
   * Lock-free reads using CopyOnWriteArrayList for zero contention.
   */
  public List<AprilTagDetection> getAllDetections() {
    detectionsCopy.clear();
    synchronized (tagLock) {
      if (detections != null && !detections.isEmpty()) {
        detectionsCopy.addAll(detections);
      }
    }

    updateCallTime();
    return detectionsCopy;
  }

  private boolean keepRunning() {
    return !stopRequested.get();
  }

  private void updateCallTime() {
    long currentTime = System.currentTimeMillis();
    lastCallInterval.set(currentTime - lastCallTime.get());
    lastCallTime.set(currentTime);
  }

  /**
   * Requests a non-blocking stop.
   */
  public void stop() {
    stopRequested.set(true);
  }

  public void run() {
    while (keepRunning()) {
      List<AprilTagDetection> newDetections = processor.getDetections();

      synchronized (tagLock) {
        detections.clear();
        if (newDetections != null && !newDetections.isEmpty()) {
          detections.addAll(newDetections);
        }
      }

      FtcUtils.sleep(Math.min(lastCallInterval.get(), MAX_UPDATE_INTERVAL));
    }

    FtcLogger.info(TAG, "Async thread stopped.");
  }
}
