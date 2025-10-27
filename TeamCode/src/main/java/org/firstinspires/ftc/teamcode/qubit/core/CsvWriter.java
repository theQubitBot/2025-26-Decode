package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class CsvWriter {
  private static final String TAG = "CsvWriter";
  private static final String COMMA = ",";
  private Writer fileWriter;
  private String line;

  public CsvWriter(String filename) {
    try {
      fileWriter = new FileWriter(
          AppUtil.ROBOT_DATA_DIR + File.separator + filename);
      Assert.assertNotNull(fileWriter, "CsvWriter>fileWriter");
      line = "";
    } catch (IOException e) {
      RobotLog.ee(TAG, e, e.getMessage());
    }
  }

  public void close() {
    FtcLogger.enter();
    try {
      if (fileWriter != null) {
        fileWriter.close();
        fileWriter = null;
      }
    } catch (IOException e) {
      RobotLog.ee(TAG, e, e.getMessage());
    }

    FtcLogger.exit();
  }

  public CsvWriter flush() {
    FtcLogger.enter();
    Assert.assertNotNull(fileWriter, "flush>fileWriter");
    try {
      fileWriter.write(line + System.lineSeparator());
      line = "";
    } catch (IOException e) {
      RobotLog.ee(TAG, e, e.getMessage());
    }

    FtcLogger.exit();
    return this;
  }

  public CsvWriter append(String data) {
    FtcLogger.enter();
    if (!line.isEmpty()) line += COMMA;
    line += data;
    FtcLogger.exit();
    return this;
  }

  public CsvWriter append(Object data) {
    return append(data.toString());
  }

  public CsvWriter append(boolean data) {
    return append(String.valueOf(data));
  }

  public CsvWriter append(byte data) {
    return append(String.valueOf(data));
  }

  public CsvWriter append(char data) {
    return append(String.valueOf(data));
  }

  public CsvWriter append(short data) {
    return append(String.valueOf(data));
  }

  public CsvWriter append(int data) {
    return append(String.valueOf(data));
  }

  public CsvWriter append(long data) {
    return append(String.valueOf(data));
  }

  public CsvWriter append(float data) {
    return append(String.valueOf(data));
  }

  public CsvWriter append(double data) {
    return append(String.valueOf(data));
  }
}
