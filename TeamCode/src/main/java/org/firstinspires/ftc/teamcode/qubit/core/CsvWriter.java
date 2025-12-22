package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

public class CsvWriter {
  private static final String TAG = "CsvWriter";
  private static final String COMMA = ",";
  private static final int MAX_LINE_BUFFER = 10000;
  private Writer fileWriter;
  private BufferedWriter bufferedWriter;
  private String line;
  private List<String> lines = new ArrayList<>();

  public CsvWriter(String filename) {
    try {
      AppUtil.ROBOT_DATA_DIR.mkdirs();
      fileWriter = new FileWriter(
          AppUtil.ROBOT_DATA_DIR + File.separator + filename);
      bufferedWriter = new BufferedWriter(fileWriter);
      line = "";
    } catch (IOException e) {
      RobotLog.ee(TAG, e, e.getMessage());
    }
  }

  public void close() {
    try {
      if (bufferedWriter != null) {
        bufferedWriter.close();
        bufferedWriter = null;
      }
    } catch (IOException e) {
      RobotLog.ee(TAG, e, e.getMessage());
    }
  }

  public CsvWriter flush() {
    try {
      for (String line : lines) {
        bufferedWriter.write(line);
        bufferedWriter.newLine(); // Platform-independent newline
      }
    } catch (IOException e) {
      RobotLog.ee(TAG, e, e.getMessage());
    } finally {
      lines.clear();
    }

    return this;
  }

  public CsvWriter append(String data) {
    if (line == null || line.isEmpty()) line = data;
    else {
      line = line + COMMA + data;
    }

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

  public CsvWriter nextLine() {
    lines.add(line);
    if (lines.size() >= MAX_LINE_BUFFER) flush();
    else line = "";

    return this;
  }
}
