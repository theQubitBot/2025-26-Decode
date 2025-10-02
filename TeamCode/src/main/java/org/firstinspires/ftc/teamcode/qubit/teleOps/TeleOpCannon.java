package org.firstinspires.ftc.teamcode.qubit.teleOps;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.qubit.core.Cannon;
import org.firstinspires.ftc.teamcode.qubit.core.AprilTag;

@TeleOp(name="CannonControl", group="TeleOp")
public class TeleOpCannon extends OpMode {
    private Cannon cannon;
    private AprilTag aprilTag;
    private int[] motifOrder;

    @Override
    public void init() {
        cannon = new Cannon(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);

        // Scan motif store result
        motifOrder = aprilTag.scanMotifOrder();
    }

    @Override
    public void loop() {
        if (gamepad1.triangle) {
            cannon.ShootGreen();
        }
        if (gamepad1.square) {
            cannon.ShootPurple();
        }
        if (gamepad1.left_trigger > 0.3) { //whatever press idk what we want
            cannon.ShootSequence(motifOrder);
        }

        // update states of cannon
        cannon.update();
    }
}
