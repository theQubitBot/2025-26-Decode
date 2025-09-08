package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcPinPointDriver;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.Locale;

@Disabled
@TeleOp(group = "TestOp")
public class GoBoDriverTeleOp extends OpMode {
  // Declare OpMode members
  private ElapsedTime runtime = null;
  private ElapsedTime loopTime = null;
  FtcPinPointDriver ftcPinPointDriver = null;
  double oldTime = 0;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    FtcLogger.enter();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    if (dashboard != null) {
      telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    ftcPinPointDriver = new FtcPinPointDriver();
    ftcPinPointDriver.init(hardwareMap, telemetry);
    FtcImu.endAutoOpHeading = 0;
    telemetry.update();
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
    FtcLogger.exit();
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    FtcLogger.enter();
    loopTime.reset();

    String position = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f",
        ftcPinPointDriver.getPositionX(DistanceUnit.INCH),
        ftcPinPointDriver.getPositionY(DistanceUnit.INCH),
        ftcPinPointDriver.getHeading(AngleUnit.DEGREES));
    telemetry.addData("Position", position);

    String velocity = String.format(Locale.US, "XVel: %.3f, YVel: %.3f, HVel: %.3f",
        ftcPinPointDriver.getVelocityX(DistanceUnit.INCH),
        ftcPinPointDriver.getVelocityY(DistanceUnit.INCH),
        ftcPinPointDriver.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
    telemetry.addData("Velocity", velocity);

    telemetry.addData("Encoders", String.format(Locale.US, "X: %d, Y: %d",
        ftcPinPointDriver.getEncoderX(), ftcPinPointDriver.getEncoderY()));

    telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
        loopTime.milliseconds(), runtime.seconds());
    telemetry.update();
    FtcLogger.exit();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    FtcLogger.enter();
    this.telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
    this.telemetry.update();
    FtcLogger.exit();
  }
}
