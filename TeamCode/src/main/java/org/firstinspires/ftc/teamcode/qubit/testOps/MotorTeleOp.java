package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcMotor;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@TeleOp(group = "TestOp")
public class MotorTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;

  static final int DELTA_SMALL = 1;
  static final int DELTA_LARGE = 10;
  static final String MOTOR_NAME = "";

  DcMotorEx motor;
  int position;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing, please wait...");
    telemetry.update();
    motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    motor.setDirection(DcMotorSimple.Direction.FORWARD);
    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData(">", "Initialization complete, Waiting for start.");
    telemetry.addData(">", "Manufacturer: %s, DeviceName: %s",
        motor.getManufacturer(), motor.getDeviceName());
    telemetry.addData(">", "%s, Port: %d",
        MOTOR_NAME, motor.getPortNumber());
    telemetry.addData(">", "Direction: %s, Position: %d",
        motor.getDirection() == DcMotor.Direction.FORWARD ? "Forward" : "Reverse",
        motor.getCurrentPosition());
    telemetry.update();
    FtcUtils.sleep(250);
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    telemetry.addData(">", "Starting...");
    telemetry.update();
    runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    FtcLogger.enter();
    // Show the elapsed game time and wheel power.
    loopTime.reset();

    double power = 0;
    if (gamepad1.dpad_up) {
      power = 0.2;
      position = 0;
    } else if (gamepad1.dpad_down) {
      power = -0.2;
      position = 0;
    }

    position = Range.clip(position, 0, 0);
    power = Range.clip(power, FtcMotor.MIN_POWER, FtcMotor.MAX_POWER);

    motor.setTargetPosition(position);
    motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    motor.setPower(power);

    telemetry.addData(">", "Use dPad up/down to positive/negative motor power");
    telemetry.addData("Motor", "Position %d, power %.2f",
        motor.getCurrentPosition(), power);
    telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    telemetry.addData(">", "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
