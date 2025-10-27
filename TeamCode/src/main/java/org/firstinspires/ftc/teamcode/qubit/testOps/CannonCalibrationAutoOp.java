package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.CsvWriter;
import org.firstinspires.ftc.teamcode.qubit.core.FtcCannon;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@Autonomous(group = "TestOp")
public class CannonCalibrationAutoOp extends LinearOpMode {
  String fileName = "cannonCalibration.txt";

  @Override
  public void runOpMode() {
    telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
    telemetry.update();
    FtcCannon cannon = new FtcCannon(null);
    cannon.init(hardwareMap, telemetry);
    telemetry.update();

    CsvWriter writer = new CsvWriter(fileName);

    while (opModeInInit()) {
      telemetry.addData(FtcUtils.TAG, "This will ramp cannons up and then down. Power and velocity data will be written to %s",
          fileName);
      telemetry.update();
      FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    while (opModeIsActive()) {
      for (double i = 1; i <= 70; i++) {
        cannon.setPower(i / 100.0);
        FtcUtils.sleep(2000);
        double power = cannon.getPower();
        double velocity = cannon.getVelocity();
        telemetry.addData(FtcUtils.TAG, "Pwr %1.2f, velocity %4.0f",
            power, velocity);
        telemetry.update();
       // writer.append(power).append(velocity).flush();
      }

      for (double i = 70; i > 1; i--) {
        cannon.setPower(i / 100.0);
        FtcUtils.sleep(2000);
        double power = cannon.getPower();
        double velocity = cannon.getVelocity();
        telemetry.addData(FtcUtils.TAG, "Pwr %1.2f, velocity %4.0f",
            power, velocity);
        telemetry.update();
        //writer.append(power).append(velocity).flush();
      }
    }

    //writer.close();
    cannon.stop();
  }
}
