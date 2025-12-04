package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface FtcSubSystemInterface {
  /**
   * Initialize standard Hardware interfaces
   *
   * @param hardwareMap The hardware map to use for initialization.
   * @param telemetry   The telemetry to use.
   * @param autoOp      If true, initializes subsystems and takes other actions that make sense only
   *                    in autoOp. You may save on processing or not move servos during teleOp init.
   */
  void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp);

  /**
   * Operate the robot/sub-system in tele operation.
   */
  void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime);

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  void start();

  /*
   * Code to run ONCE after the driver hits STOP
   */
  void stop();

  void disableTelemetry();

  void enableTelemetry();

  void showTelemetry();
}
