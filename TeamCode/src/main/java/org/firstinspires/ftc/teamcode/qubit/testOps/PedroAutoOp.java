package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.TrollBots.BaseBot;

@Disabled
@Autonomous(group = "TestOp")
public class PedroAutoOp extends LinearOpMode {
  BaseBot robot = null;
  Follower follower;
  PedroTestGoal pedroGoal;

  @Override
  public void runOpMode() {
    telemetry.addData(FtcUtils.TAG, "Initializing. Please wait...");
    telemetry.update();

    robot = BaseBot.getBot();
    follower = Constants.createFollower(hardwareMap);
    pedroGoal = new PedroTestGoal(this, robot, follower).init();

    while (opModeInInit()) {
      telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play.");
      telemetry.update();
      FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    pedroGoal.execute();
  }
}
