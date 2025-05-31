package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class FtcSubSystemBase implements FtcSubSystemInterface {
  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    throw new UnsupportedOperationException();
  }

  public void start() {
    throw new UnsupportedOperationException();
  }

  public void operate(Gamepad gamePad1, Gamepad gamePad2) {
    throw new UnsupportedOperationException();
  }

  public void stop() {
    throw new UnsupportedOperationException();
  }

  public void showTelemetry() {
    throw new UnsupportedOperationException();
  }
}
