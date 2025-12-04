package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class FtcSubSystemBase implements FtcSubSystemInterface {
  protected String TAG = "You must set TAG string in your syb system constructor.";
  protected boolean telemetryEnabled = true;
  protected Telemetry telemetry = null;
  protected HardwareMap hardwareMap = null;

  public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
    throw new UnsupportedOperationException();
  }

  public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
    throw new UnsupportedOperationException();
  }

  public void start() {
    throw new UnsupportedOperationException();
  }

  public void stop() {
    throw new UnsupportedOperationException();
  }

  public void disableTelemetry() {
    throw new UnsupportedOperationException();
  }

  public void enableTelemetry() {
    throw new UnsupportedOperationException();
  }

  public void showTelemetry() {
    throw new UnsupportedOperationException();
  }
}
