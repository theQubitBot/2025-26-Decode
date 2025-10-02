package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Cannon {
    private final DcMotorEx greenShooter;
    private final DcMotorEx purpleShooter;
    private final Servo feeder;
    // Feeder positions tune these to whatever
    private static final double FEEDER_HOLD = 0.0;
    private static final double FEEDER_PUSH = 1.0;
    // Timing constants tune these to whatever
    private static final long FEED_TIME = 250;
    private static final long BETWEEN_SHOTS = 500;
    // machine tracking state I think this helps
    private boolean feeding = false;
    private long feedStartTime = 0;
    private boolean shootingSequence = false;
    private int[] sequence;
    private int sequenceIndex = 0;
    private long lastShotTime = 0;

    public Cannon(HardwareMap hwMap) {
        greenShooter = hwMap.get(DcMotorEx.class, "greenShooter");
        purpleShooter = hwMap.get(DcMotorEx.class, "purpleShooter");
        feeder = hwMap.get(Servo.class, "feeder");
        feeder.setPosition(FEEDER_HOLD);
    }
    public void ShootGreen() {
        greenShooter.setPower(1.0);
        startFeeder();
    }
    public void ShootPurple() {
        purpleShooter.setPower(1.0);
        startFeeder();
    }
    public void ShootSequence(int[] motifOrder) {
        if (!shootingSequence) { // only start if not already running
            sequence = motifOrder;
            sequenceIndex = 0;
            shootingSequence = true;
        }            lastShotTime = System.currentTimeMillis() - BETWEEN_SHOTS; // allow immediate first shot

    }

    public void update() {
        long now = System.currentTimeMillis(); //I think this is milliseconds

        // handle feeder whether it needs to be reset or not
        if (feeding && now - feedStartTime > FEED_TIME) {
            feeder.setPosition(FEEDER_HOLD);
            feeding = false;
        }
        // handle sequence of balls and shooting them
        if (shootingSequence) {
            if (sequenceIndex < sequence.length) {
                if (now - lastShotTime > BETWEEN_SHOTS) { //has there been enough time
                    // fire next ball
                    if (sequence[sequenceIndex] == 0) {
                        ShootGreen();
                    } else {
                        ShootPurple();
                    }
                    sequenceIndex+=1;
                    lastShotTime = now;
                }
            } else {
                shootingSequence = false; // done
            }
        }
    }

    // helper for the starting time feeder thingy
    private void startFeeder() {
        feeder.setPosition(FEEDER_PUSH);
        feeding = true;
        feedStartTime = System.currentTimeMillis();
    }
}
//all the constants need to be fine-tuned