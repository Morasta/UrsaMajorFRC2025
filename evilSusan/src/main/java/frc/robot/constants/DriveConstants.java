package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public class DriveConstants {
            public static enum kWheels {
            frontLeft
            , frontRight
            , rearLeft
            , rearRight
        };

        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveRearTurnSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 0.5;

        public static final int kFrontLeftVel = 1;
        public static final int kRearLeftVel = 1;
        public static final int kPFrontRightVel = 1;
        public static final int kPRearRightVel = 1;

        //Drivetrain motor ports
        public static final int kFrontLeftMotorPort = 1; 
        public static final int kFrontRightMotorPort = 13;
        public static final int kRearLeftMotorPort = 3;
        public static final int kRearRightMotorPort = 2;
        
        public static final int kFrontLeftEncoderPortA = 0;
        public static final int kFrontLeftEncoderPortB = 1;
        public static final int kFrontRightEncoderPortA = 2;
        public static final int kFrontRightEncoderPortB = 3;
        public static final int kRearLeftEncoderPortA = 4;
        public static final int kRearLeftEncoderPortB = 5;
        public static final int kRearRightEncoderPorts = 6;
        public static final int kRearLeftEncoderPorts = 7;

        public static final boolean kFrontRightEncoderReversed = true;
        public static final boolean kFrontLeftEncoderReversed = false;
        
        public static final boolean kRearRightEncoderReversed = true;
        public static final boolean kRearLeftEncoderReversed = false;

        
        // TODO: do the math for this, such as: (kWheelDiameterMeters * Math.PI) / kEncoderCPR;
        // Assumes the encoders are directly mounted on the wheel shafts
        public static final double kEncoderDistancePerPulse = 1.0;
        public static final int[] kEncoderDistancePerPulset = new int[] {0, 1};

        public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(1, 0.8, 0.15);

        public static final double kTrackWidth = 0.5;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        // Distance between centers of front and back wheels on robot

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );
}
