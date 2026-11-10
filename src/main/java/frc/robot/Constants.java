package frc.robot;

public final class Constants {

    public static final class RobotModeConstants {
   
        public static final boolean IS_ATTACHMENTS_MOUNTED = false;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorCanId = 13;
        public static final int kBeamBreakDioChannel = 0;
        public static final double kIntakeVoltage = 8.0;
        public static final double kEjectVoltage = -6.0;
    }

    public static final class ArmConstants {
        public static final int kArmMotorCanId = 14;
        public static final double kArmGearRatio = 50.0;

        public static final double kP = 12.0;
        public static final double kI = 0.0;
        public static final double kD = 0.5;
        public static final double kG = 0.4;

        public static final double kMinPositionRotations = 0.0;
        public static final double kMaxPositionRotations = 0.305;
    }

    public static final class ElevatorConstants {
        public static final int kElevatorMotorCanId = 16;
        public static final double kElevatorGearRatio = 10.0;
        public static final double kDrumRadiusMeters = 0.025;

        public static final double kP = 20.0;
        public static final double kI = 0.0;
        public static final double kD = 0.2;
        public static final double kG = 0.6;

        public static final double kMinHeightRotations = 0.0;
        public static final double kMaxHeightRotations = 9.55;
    }

    public static final class ShooterConstants {
        public static final int kMasterCanId = 17;
        public static final int kFollowerCanId = 18;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;

        public static final double kTargetRPS = 75.0;
        public static final double kRPSTolerance = 2.0;
    }

    public static final class VisionConstants {
        public static final String kLimelightName = "limelight-chassis";

        public static final double kForwardMeters = 0.25;
        public static final double kSideMeters = 0.0;
        public static final double kUpMeters = 0.45;

        public static final double kRollDegrees = 0.0;
        public static final double kPitchDegrees = 25.0;
        public static final double kYawDegrees = 0.0;
    }
}