package org.firstinspires.ftc.teamcode.qubit.core;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Arrays;
public class AprilTag {
    //placeholders for HuskyLens SDK or whatever it is
    //Each tag encodes the MOTIF
    private int[]  motifOrder = new int[]{0,0,0};
    public AprilTag(HardwareMap hwMap) {
        try {
            RobotLog.i("April Tag: HuskyLens Initialized");
        } catch (Exception e) {
            RobotLog.e("AprilTag: Camera initialization failed -> " + e.getMessage());
        }
    }
    public int[] scanMotifOrder() {
        // HuskyLensResult[] results = huskyLens.getBlocks();
        // int detectedTagId = results[0].id;
        int detectedTagId = 1; //idk how to actually transfer the data so this is placeholder
        motifOrder = mapTagToMotif(detectedTagId);
        RobotLog.i("AprilTag: Scanned motif = " + Arrays.toString(motifOrder));

        return motifOrder;
    }
    public int[] getMotifOrder() {
        return motifOrder;
    }
    private int[] mapTagToMotif(int tagId) {
        switch (tagId) {
            case 21:
                return new int[]{0,1,1}; //Green, Purple, Purple
            case 22:
                return new int[]{1,0,1}; // Purple, Green, Purple
            case 23:
                return new int[]{1,1,0}; // Purple, Purple, Green
            default:
                return new int[]{1,1,1}; //worst case scenario go Purple, Purple, Purple
        }
    }
}
