package com.spartronics4915.frc;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Swerve {
        public static final double kWheelbase = Units.inchesToMeters(23.75);
        public static final double kTrackWidth = Units.inchesToMeters(18.75);

        public static final double kRadius = Math.hypot(kWheelbase / 2, kTrackWidth / 2);

        public static final double kMaxSpeed = 1; // Units.feetToMeters(14.5);
        public static final double kMaxAngularSpeed = kMaxSpeed / kRadius;

        public static final double kDeadband = 0.05;
        public static final double kResponseCurveExponent = 3;
    }
}
