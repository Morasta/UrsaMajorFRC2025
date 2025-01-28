package frc.robot;

import java.lang.reflect.Array;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    // Constant definitions for controller mapping used by the "operator" or driver
    // This should map to the controller classes we'll define later
    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.5;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class DriveConstants {
        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 0.5;

        public static final int kFrontLeftVel = 1;
        public static final int kRearLeftVel = 1;
        public static final int kPFrontRightVel = 1;
        public static final int kPRearRightVel = 1;

        public static final int kFrontLeftMotorPort = 0;
        public static final int kFrontRightMotorPort = 1;
        public static final int kRearLeftMotorPort = 2;
        public static final int kRearRightMotorPort = 3;

        public static final int[] kFrontLeftEncoderPorts = new int[] { 0, 1 };
        public static final int[] kRearLeftEncoderPorts = new int[] { 2, 3 };
        public static final int[] kFrontRightEncoderPorts = new int[] { 4, 5 };
        public static final int[] kRearRightEncoderPorts = new int[] { 6, 7 };

        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kRearLeftEncoderReversed = true;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kRearRightEncoderReversed = true;

        // TODO: do the math for this, such as: (kWheelDiameterMeters * Math.PI) /
        // kEncoderCPR;
        // Assumes the encoders are directly mounted on the wheel shafts
        public static final double kEncoderDistancePerPulse = 1.0;

        public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(1, 0.8, 0.15);

        public static final double kTrackWidth = 0.5;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        // Distance between centers of front and back wheels on robot

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

}
