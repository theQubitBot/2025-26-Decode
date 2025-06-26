package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

@Disabled
@Autonomous(group = "TestOp", preselectTeleOp = "DataPersistenceTeleOp")
public class DataPersistenceAutoOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    FtcLogger.enter();
    telemetry.addData(">", "Initializing. Please wait...");
    telemetry.update();

    // Wait for driver to hit start
    while (opModeInInit()) {
      telemetry.addData(">", "Initialization complete. Waiting for start");
      telemetry.update();
      FtcUtils.sleep(10);
    }

    telemetry.addData(">", "Updating config. Please wait...");
    telemetry.update();
    FtcImu.endAutoOpHeading = 47;
    telemetry.addData("config", "endAutoOpHeading=%.1f",
        FtcImu.endAutoOpHeading);
    telemetry.addData("config", "Update successful");
    telemetry.update();
    while (opModeIsActive()) {
      FtcUtils.sleep(50);
    }

    FtcLogger.exit();
  }
}
