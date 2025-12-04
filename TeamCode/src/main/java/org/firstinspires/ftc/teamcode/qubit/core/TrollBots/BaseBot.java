package org.firstinspires.ftc.teamcode.qubit.core.TrollBots;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.qubit.core.FtcAprilTag;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBlinkinLed;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBulkRead;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcDriveTrain;
import org.firstinspires.ftc.teamcode.qubit.core.FtcIntake;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcSorter;
import org.firstinspires.ftc.teamcode.qubit.core.FtcSubSystemBase;
import org.firstinspires.ftc.teamcode.qubit.core.LlArtifactSensor;
import org.firstinspires.ftc.teamcode.qubit.core.MatchConfig;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TrollBotEnum;

import java.util.Arrays;
import java.util.List;

/**
 * A class to manage the robot. This is a composite design pattern.
 * Robot level operations are simply invoked on all subsystems.
 */
public abstract class BaseBot extends FtcSubSystemBase {
  public static final TrollBotEnum trollBot = TrollBotEnum.TrollBotA;
  // robot sub systems
  public FtcAprilTag aprilTag = null;
  public LlArtifactSensor artifactSensor = null;
  public FtcBlinkinLed blinkinLed = null;
  public FtcBulkRead bulkRead = null;
  public FtcCannon cannon = null;
  public MatchConfig config = null;
  public FtcDriveTrain driveTrain = null;
  public FtcIntake intake = null;
  public FtcSorter sorter = null;

  public static BaseBot getBot() {
    BaseBot bot = null;
    switch (trollBot) {
      case Unknown:
      case TrollBotK:
        break;
      case TrollBotA:
        bot = new BotA();
        break;
      case TrollBotB:
        bot = new BotB();
        break;
      case TrollBotC:
        bot = new BotC();
        break;
      case TrollBotD:
        bot = new BotD();
        break;
      case TrollBotL:
        bot = new BotL();
        break;
    }

    return bot;
  }

  public static List<DcMotorSimple.Direction> getMotorDirections() {
    List<DcMotorSimple.Direction> motorDirections = null;
    // leftFront, leftRear, rightFront, rightRear

    switch (trollBot) {
      case Unknown:
      case TrollBotK:
        break;
      case TrollBotA:
        motorDirections = Arrays.asList(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        break;
      case TrollBotB:
        motorDirections = Arrays.asList(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        break;
      case TrollBotC:
        motorDirections = Arrays.asList(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        break;
      case TrollBotD:
        motorDirections = Arrays.asList(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        break;
      case TrollBotL:
        motorDirections = Arrays.asList(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        break;
    }

    return motorDirections;
  }

  /**
   * Display game pad telemetry.
   *
   * @param gamePad The gamePad.
   */
  public void showGamePadTelemetry(Gamepad gamePad) {
    FtcLogger.enter();
    if (telemetryEnabled && telemetry != null) {
      telemetry.addData("LeftStick", "%.2f %.2f",
          gamePad.left_stick_x, gamePad.left_stick_y);
      telemetry.addData("RightStick", "%.2f, %.2f",
          gamePad.right_stick_x, gamePad.right_stick_y);
    }

    FtcLogger.exit();
  }
}
