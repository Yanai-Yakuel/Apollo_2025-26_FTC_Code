package org.firstinspires.ftc.teamcode.apollo.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Encoder class wraps a DcMotorEx to provide velocity and position with optional direction inversion.
 * Useful for tracking wheels.
 */
public class Encoder {
    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;
    private ElapsedTime clock;
    private Direction direction;

    private int lastPosition;
    private double lastVelocity;

    public Encoder(DcMotorEx motor) {
        this(motor, new ElapsedTime());
    }

    public Encoder(DcMotorEx motor, ElapsedTime clock) {
        this.motor = motor;
        this.clock = clock;
        this.direction = Direction.FORWARD;

        this.lastPosition = 0;
        this.lastVelocity = 0;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int multiplier = direction.getMultiplier();
        int currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime; // Simplified if not tracking dt manually
            lastPosition = currentPosition;
        }
        return currentPosition;
    }

    public double getRawVelocity() {
        int multiplier = direction.getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    public double getCorrectedVelocity() {
        return (double) motor.getVelocity();
    }
}
