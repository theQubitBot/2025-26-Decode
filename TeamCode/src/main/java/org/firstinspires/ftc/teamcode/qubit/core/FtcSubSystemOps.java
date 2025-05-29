package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface FtcSubSystemOps {
  void init(HardwareMap hardwareMap, Telemetry telemetry);
  void start();
  void operate(Gamepad gamePad1, Gamepad gamePad2);
  void stop();
  void showTelemetry();
}
