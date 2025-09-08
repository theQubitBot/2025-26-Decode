package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class FtcPinPointDriver extends FtcSubSystemBase {
  private static final String TAG = "FtcPinPointDriver";
  public boolean telemetryEnabled = true;
  Telemetry telemetry = null;
  private GoBildaPinpointDriver goboDriver;
  private Deadline deadline;

  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    FtcLogger.enter();
    this.telemetry = telemetry;
    goboDriver = hardwareMap.get(GoBildaPinpointDriver.class, "goboDriver");
    if (goboDriver != null) {
      goboDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
      goboDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
          GoBildaPinpointDriver.EncoderDirection.FORWARD);
      goboDriver.resetPosAndIMU();
      goboDriver.update();
      deadline = new Deadline(1, TimeUnit.MILLISECONDS);
    }

    FtcLogger.exit();
  }

  public int getEncoderX() {
    refreshData();
    return goboDriver.getEncoderX();
  }

  public int getEncoderY() {
    refreshData();
    return goboDriver.getEncoderY();
  }

  /**
   * @return Heading in requested units.
   */
  public double getHeading(AngleUnit angleUnit) {
    refreshData();
    return FtcImu.normalize(goboDriver.getHeading(angleUnit), angleUnit);
  }

  /**
   * @return Heading velocity in requested units / second
   */
  public double getHeadingVelocity(UnnormalizedAngleUnit angleUnit) {
    refreshData();
    return goboDriver.getHeadingVelocity(angleUnit);
  }

  /**
   * @return the estimated X (forward) position of the robot in requested units
   */
  public double getPositionX(DistanceUnit distanceUnit) {
    refreshData();
    return goboDriver.getPosX(distanceUnit);
  }

  /**
   * @return the estimated Y (Strafe) position of the robot in requested units
   */
  public double getPositionY(DistanceUnit distanceUnit) {
    refreshData();
    return goboDriver.getPosY(distanceUnit);
  }

  /**
   * @return the estimated X (forward) velocity of the robot in requested units/sec
   */
  public double getVelocityX(DistanceUnit distanceUnit) {
    refreshData();
    return goboDriver.getVelX(distanceUnit);
  }

  /**
   * @return the estimated Y (strafe) velocity of the robot in requested units/sec
   */
  public double getVelocityY(DistanceUnit distanceUnit) {
    refreshData();
    return goboDriver.getVelY(distanceUnit);
  }

  public void refreshData() {
    if (deadline.hasExpired()) {
      goboDriver.update();
      deadline.reset();
    }
  }

  /**
   * Display IMU (gyro) telemetry.
   */
  public void showTelemetry() {
    FtcLogger.enter();
    if (telemetryEnabled) {
      telemetry.addData("Heading/Yaw", "%.2f", getHeading(AngleUnit.DEGREES));
    }

    FtcLogger.exit();
  }
}
