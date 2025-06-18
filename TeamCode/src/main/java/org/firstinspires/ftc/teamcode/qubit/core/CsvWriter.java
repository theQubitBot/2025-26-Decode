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

    public void flush() {
        FtcLogger.enter();
        Assert.assertNotNull(fileWriter, "flush>fileWriter");
        try {
            fileWriter.write(line + System.lineSeparator());
            line = "";
        } catch (IOException e) {
            RobotLog.ee(TAG, e, e.getMessage());
        }

        FtcLogger.exit();
    }

    public void append(String data) {
        FtcLogger.enter();
        if (!line.equals("")) line += COMMA;
        line += data;
        FtcLogger.exit();
    }

    public void append(Object data) {
        append(data.toString());
    }

    public void append(boolean data) {
        append(String.valueOf(data));
    }

    public void append(byte data) {
        append(String.valueOf(data));
    }

    public void append(char data) {
        append(String.valueOf(data));
    }

    public void append(short data) {
        append(String.valueOf(data));
    }

    public void append(int data) {
        append(String.valueOf(data));
    }

    public void append(long data) {
        append(String.valueOf(data));
    }

    public void append(float data) {
        append(String.valueOf(data));
    }

    public void append(double data) {
        append(String.valueOf(data));
    }
}
