package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;

/**
 * A simple TeleOp to tune a motor's velocity PIDF coefficients using FTC Dashboard.
 *
 * <p>To use, set a target velocity using the 'targetVelocity' variable, then adjust the
 * PIDF coefficients in the FTC Dashboard to minimize the difference between the target
 * and the motor's actual velocity.
 * d=37.5, v=1040, aligned with obelisk
 * orgPIDF = 10 3 0 0
 */
//@Disabled
@TeleOp(group = "TestOp")
@Config
public class MotorVelocityPidTuner extends LinearOpMode {
  // These variables can be changed in FTC Dashboard when the OpMode is initialized.
  public static double targetVelocity = 1060; // Target speed in ticks per second
  public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 0);
  private PIDFCoefficients pidfOld = new PIDFCoefficients(0, 0, 0, 0);
  private VoltageSensor batteryVoltageSensor;

  FtcCannon cannon;

  @Override
  public void runOpMode() {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    cannon = new FtcCannon(null);
    cannon.init(hardwareMap, telemetry);

    // Get the original coefficients to potentially restore them later
    PIDFCoefficients originalPidf = cannon.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
    telemetry.addData("Status", "Initialized");
    telemetry.addData("Instructions", "Use FTC Dashboard to tune PIDF.");
    telemetry.addData("PIDF original", "%.2f %.2f %.2f %.2f",
        originalPidf.p, originalPidf.i, originalPidf.d, originalPidf.f);
    telemetry.update();

    // start with the sdk defaults
    MOTOR_VELO_PID.p = originalPidf.p;
    MOTOR_VELO_PID.i = originalPidf.i;
    MOTOR_VELO_PID.d = originalPidf.d;
    MOTOR_VELO_PID.f = originalPidf.f;

    waitForStart();

    while (opModeIsActive()) {
      if (MOTOR_VELO_PID.p != pidfOld.p || MOTOR_VELO_PID.i != pidfOld.i ||
          MOTOR_VELO_PID.d != pidfOld.d || MOTOR_VELO_PID.f != pidfOld.f) {
        // Update the PIDF coefficients on the motor controller from the FTC Dashboard values
//        cannon.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
//            new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i,
//                MOTOR_VELO_PID.d, MOTOR_VELO_PID.f));
        pidfOld.p = MOTOR_VELO_PID.p;
        pidfOld.i = MOTOR_VELO_PID.i;
        pidfOld.d = MOTOR_VELO_PID.d;
        pidfOld.f = MOTOR_VELO_PID.f;
      }

      // Set the target velocity
      cannon.setVelocity(targetVelocity, false);

      // Display actual motor velocity and PIDF coefficients on the driver station
      telemetry.addData("Target Velocity", targetVelocity);
      telemetry.addData("Actual Velocity", cannon.getVelocity());
      telemetry.addData("PIDF", "%f %f %f %f",
          MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f);

      telemetry.update();
    }

    // Stop the motor and restore original coefficients on exit
    cannon.setVelocity(FtcCannon.LEFT_CANNON_ZERO_VELOCITY, false);
//    cannon.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, originalPidf);
  }
}
