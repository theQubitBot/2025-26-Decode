package org.firstinspires.ftc.teamcode.qubit.motorOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;

@Disabled
@Config
@TeleOp(group = "TestOp")
public class MotorVelocityPIDTuner extends LinearOpMode {
  public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 1, 12.5);

  private final FtcDashboard dashboard = FtcDashboard.getInstance();

  private VoltageSensor batteryVoltageSensor;

  @Override
  public void runOpMode() throws InterruptedException {
    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    DcMotorEx leftCannonMotor = hardwareMap.get(DcMotorEx.class, FtcCannon.LEFT_CANNON_MOTOR_NAME);
    MotorConfigurationType motorConfigurationType = leftCannonMotor.getMotorType().clone();
    motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
    leftCannonMotor.setMotorType(motorConfigurationType);
    leftCannonMotor.setDirection(DcMotorEx.Direction.FORWARD);
    leftCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    leftCannonMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    setPIDFCoefficients(leftCannonMotor, MOTOR_VELO_PID);

    DcMotorEx rightCannonMotor = hardwareMap.get(DcMotorEx.class, FtcCannon.RIGHT_CANNON_MOTOR_NAME);
    motorConfigurationType = rightCannonMotor.getMotorType().clone();
    motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
    rightCannonMotor.setMotorType(motorConfigurationType);
    rightCannonMotor.setDirection(DcMotorEx.Direction.REVERSE);
    rightCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    rightCannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    setPIDFCoefficients(rightCannonMotor, MOTOR_VELO_PID);

    TuningController tuningController = new TuningController();

    double lastKp = 0.0;
    double lastKi = 0.0;
    double lastKd = 0.0;
    double lastKf = getMotorVelocityF();

    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    telemetry.addLine("Ready!");
    telemetry.update();
    telemetry.clearAll();

    waitForStart();
    if (isStopRequested()) return;
    tuningController.start();

    while (!isStopRequested() && opModeIsActive()) {
      double targetVelocity = tuningController.getTargetVelocity();
      leftCannonMotor.setVelocity(targetVelocity);
      rightCannonMotor.setVelocity(targetVelocity);

      telemetry.addData("targetVelocity", targetVelocity);

      double motorVelocity = leftCannonMotor.getVelocity();
      telemetry.addData("velocity", motorVelocity);
      telemetry.addData("error", targetVelocity - motorVelocity);

      telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
      telemetry.addData("lowerBound", 0);

      if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
        setPIDFCoefficients(leftCannonMotor, MOTOR_VELO_PID);
        setPIDFCoefficients(rightCannonMotor, MOTOR_VELO_PID);

        lastKp = MOTOR_VELO_PID.p;
        lastKi = MOTOR_VELO_PID.i;
        lastKd = MOTOR_VELO_PID.d;
        lastKf = MOTOR_VELO_PID.f;
      }

      telemetry.update();
    }
  }

  private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
    motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
        coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
    ));
  }

  public static double getMotorVelocityF() {
    // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
    return 32767 * 60.0 / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV);
  }
}
