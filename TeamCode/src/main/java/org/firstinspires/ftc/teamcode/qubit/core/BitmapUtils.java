package org.firstinspires.ftc.teamcode.qubit.core;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

/**
 * Global utility functions
 */
public final class BitmapUtils {
  private static final String TAG = "BitmapUtils";
  private final Telemetry telemetry;

  /**
   * State regarding where and how to save frames when the 'A' button is pressed.
   */
  private static int captureCounter = 0;

  /* Constructor */
  public BitmapUtils(Telemetry telemetry) {
    this.telemetry = telemetry;
  }

  public void saveBitmap(Bitmap bitmap) {
    FtcLogger.enter();
    File file = new File(AppUtil.ROBOT_DATA_DIR, String.format(Locale.getDefault(),
        "ftc-image-%d.jpg", captureCounter++));
    try {
      try (FileOutputStream outputStream = new FileOutputStream(file)) {
        bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
        telemetry.addData(TAG, "Saved %s", file.getName());
        FtcLogger.debug("Saved %s", file.getName());
      }
    } catch (IOException e) {
      String message = String.format("Exception saving %s", file.getName());
      RobotLog.ee(TAG, e, message);
      telemetry.addData(TAG, message);
    }

    FtcLogger.exit();
  }
}
