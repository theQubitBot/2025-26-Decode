package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

@Disabled
@TeleOp(group = "TestOp")
public class DriveTrainTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  private double lastLoopTime = 0.0;
  FtcBot robot = null;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();
    robot = new FtcBot();
    robot.init(hardwareMap, telemetry, false);
    robot.driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    robot.driveTrain.setDriveTypeAndMode(
        DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
    telemetry.update();
    FtcUtils.sleep(FtcUtils.CYCLE_MS);
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    FtcLogger.enter();
    telemetry.addData(FtcUtils.TAG, "Starting...");
    telemetry.update();
    runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    if (FtcUtils.DEBUG) {
      robot.enableTelemetry();
    } else {
      robot.disableTelemetry();
    }

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

    telemetry.addData(FtcUtils.TAG, "a: FWD POV, b: RWD POV, x: MecanumDrive FOD, y: AWD POV");
    if (gamepad1.aWasPressed()) {
      robot.driveTrain.setDriveTypeAndMode(
          DriveTrainEnum.FRONT_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
    } else if (gamepad1.bWasPressed()) {
      robot.driveTrain.setDriveTypeAndMode(
          DriveTrainEnum.REAR_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
    } else if (gamepad1.xWasPressed()) {
      robot.driveTrain.setDriveTypeAndMode(
          DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.FIELD_ORIENTED_DRIVE);
    } else if (gamepad1.yWasPressed()) {
      robot.driveTrain.setDriveTypeAndMode(
          DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
    }

    robot.bulkRead.clearBulkCache();
    robot.driveTrain.operate(gamepad1, gamepad2, lastLoopTime, runtime);
    robot.driveTrain.showTelemetry();
    robot.showGamePadTelemetry(gamepad1);
    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    lastLoopTime = loopTime.milliseconds();
    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    robot.driveTrain.stop();
    telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    telemetry.update();
    FtcLogger.exit();
  }
}
