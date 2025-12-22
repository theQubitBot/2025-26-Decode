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
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;

@Disabled
@Config
@TeleOp(group = "TestOp")
public class MotorPositionPIDTuner extends LinearOpMode {
  public static PIDFCoefficients MOTOR_POS_PID = new PIDFCoefficients(5.0, 0.0, 0.0, 0.0);
  public static double MOTOR_POWER = 1.0;
  public static int LARGE_INCREMENT = 500;
  public static int SMALL_INCREMENT = 100;

  private final FtcDashboard dashboard = FtcDashboard.getInstance();

  enum Mode {
    AUTOMATED,
    MANUAL
  }

  @Override
  public void runOpMode() throws InterruptedException {
    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    DcMotorEx leftCannonMotor = hardwareMap.get(DcMotorEx.class, FtcCannon.LEFT_CANNON_MOTOR_NAME);
    MotorConfigurationType motorConfigurationType = leftCannonMotor.getMotorType().clone();
    motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
    leftCannonMotor.setMotorType(motorConfigurationType);
    leftCannonMotor.setDirection(DcMotorEx.Direction.FORWARD);
    leftCannonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    leftCannonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftCannonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    setPIDFCoefficients(leftCannonMotor, MOTOR_POS_PID);

    MotorPositionTuningController positionController = new MotorPositionTuningController();

    Mode mode = Mode.AUTOMATED;
    int manualTargetPosition = 0;

    double lastKp = 0.0;
    double lastKi = 0.0;
    double lastKd = 0.0;
    double lastKf = 0.0;

    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    telemetry.addLine("Y: Toggle mode");
    telemetry.addLine("Manual mode controls:");
    telemetry.addLine("  DPad Up/Down: Large increment (+/-" + LARGE_INCREMENT + ")");
    telemetry.addLine("  DPad Left/Right: Small increment (+/-" + SMALL_INCREMENT + ")");
    telemetry.addLine("  A: Reset to home (0)");
    telemetry.update();
    telemetry.clearAll();

    waitForStart();
    if (isStopRequested()) return;
    positionController.start();

    while (!isStopRequested() && opModeIsActive()) {
      // Handle mode toggle
      if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
        mode = (mode == Mode.AUTOMATED) ? Mode.MANUAL : Mode.AUTOMATED;
        if (mode == Mode.AUTOMATED) {
          positionController.start(); // Reset automated sequence when switching back
        } else {
          manualTargetPosition = leftCannonMotor.getCurrentPosition(); // Start from current position
        }
      }

      // Get target position based on mode
      int targetPosition;
      if (mode == Mode.AUTOMATED) {
        targetPosition = positionController.getTargetPosition();
      } else {
        // Manual mode controls
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
          manualTargetPosition += LARGE_INCREMENT;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
          manualTargetPosition -= LARGE_INCREMENT;
        } else if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
          manualTargetPosition += SMALL_INCREMENT;
        } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
          manualTargetPosition -= SMALL_INCREMENT;
        } else if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
          manualTargetPosition = 0; // Reset to home
        }

        targetPosition = manualTargetPosition;
      }

      // Set target positions and power
      leftCannonMotor.setTargetPosition(targetPosition);
      leftCannonMotor.setPower(MOTOR_POWER);

      // Update telemetry
      telemetry.addData("mode", mode);
      if (mode == Mode.AUTOMATED) {
        telemetry.addData("state", positionController.getCurrentState());
      }

      telemetry.addData("targetPosition", targetPosition);

      int currentPosition = leftCannonMotor.getCurrentPosition();
      telemetry.addData("position", currentPosition);
      telemetry.addData("error", targetPosition - currentPosition);

      telemetry.addData("upperBound", MotorPositionTuningController.TESTING_MAX_POSITION);
      telemetry.addData("lowerBound", MotorPositionTuningController.TESTING_MIN_POSITION);

      telemetry.addData("PIDF", "P=%.2f, I=%.2f, D=%.2f, F=%.2f",
          MOTOR_POS_PID.p, MOTOR_POS_PID.i, MOTOR_POS_PID.d, MOTOR_POS_PID.f);

      // Update PIDF if changed
      if (lastKp != MOTOR_POS_PID.p || lastKi != MOTOR_POS_PID.i ||
          lastKd != MOTOR_POS_PID.d || lastKf != MOTOR_POS_PID.f) {
        setPIDFCoefficients(leftCannonMotor, MOTOR_POS_PID);
        lastKp = MOTOR_POS_PID.p;
        lastKi = MOTOR_POS_PID.i;
        lastKd = MOTOR_POS_PID.d;
        lastKf = MOTOR_POS_PID.f;
      }

      telemetry.update();
    }

    // Stop motors when done
    leftCannonMotor.setPower(0);
  }

  private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
    motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(
        coefficients.p, coefficients.i, coefficients.d, coefficients.f
    ));
  }
}
