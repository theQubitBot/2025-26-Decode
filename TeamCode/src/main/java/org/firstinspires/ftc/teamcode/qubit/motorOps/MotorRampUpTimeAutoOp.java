package org.firstinspires.ftc.teamcode.qubit.motorOps;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.CsvWriter;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;

@Disabled
@Config
@TeleOp(group = "TestOp")
public class MotorRampUpTimeAutoOp extends LinearOpMode {
  public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(60, 0.25, 2, 13.0);

  private VoltageSensor batteryVoltageSensor;

  @SuppressLint("DefaultLocale")
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

    String filename = "motorRampUp.txt";
    telemetry.addLine("Ready!");
    telemetry.update();

    waitForStart();
    if (isStopRequested()) return;

    double targetVelocity = 1500;
    CsvWriter writer = new CsvWriter(filename);
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    leftCannonMotor.setVelocity(targetVelocity);
    rightCannonMotor.setVelocity(targetVelocity);

    while (!isStopRequested() && opModeIsActive()) {
      double motorVelocity = leftCannonMotor.getVelocity();
      writer.append(String.format("%.6f", runtime.milliseconds()))
          .append(String.format("%.0f", targetVelocity))
          .append(String.format("%.0f", motorVelocity))
          .nextLine();
      if (motorVelocity >= targetVelocity) break;
    }

    leftCannonMotor.setVelocity(0);
    rightCannonMotor.setVelocity(0);
    writer.flush().close();
    telemetry.addLine("Done!");
    telemetry.update();
  }

  private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
    motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
        coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
    ));
  }
}
