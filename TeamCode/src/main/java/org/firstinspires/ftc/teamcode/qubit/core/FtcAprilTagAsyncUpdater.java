package org.firstinspires.ftc.teamcode.qubit.core;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * A class to asynchronously execute the processor.
 */
public class FtcAprilTagAsyncUpdater implements Runnable {
  public static final String TAG = "FtcSorterAsyncUpdater";
  private final AprilTagProcessor processor;
  private boolean stopRequested;
  private List<AprilTagDetection> detections, detectionsCopy;

  public FtcAprilTagAsyncUpdater(AprilTagProcessor processor) {
    FtcLogger.enter();
    this.processor = processor;
    stopRequested = false;
    detections = null;
    detectionsCopy = null;
    FtcLogger.exit();
  }

  /**
   * Returns a copy of current detections.
   */
  public List<AprilTagDetection> getAllDetections() {
    detectionsCopy.clear();
    synchronized (processor) {
      if (detections != null && !detections.isEmpty()) {
        detectionsCopy.addAll(detections);
      }
    }

    return detectionsCopy;
  }

  private synchronized boolean keepRunning() {
    return !stopRequested;
  }

  /**
   * Requests a non-blocking stop.
   */
  public synchronized void stop() {
    stopRequested = true;
  }

  public void run() {
    while (keepRunning()) {
      synchronized (processor) {
        detections = processor.getDetections();
      }

      FtcUtils.sleep(1);
    }

    FtcLogger.info(TAG, "Async thread stopped.");
  }
}
