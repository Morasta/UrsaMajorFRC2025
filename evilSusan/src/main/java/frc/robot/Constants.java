package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public final class Constants {
    // Constant definitions for controller mapping used by the "operator" or driver
    // This should map to the controller classes we'll define later
    public static final class OIConstants {
        //Joystick ports
        public static final int kDriverJoystickPort = 0;
        public static final int kClawJoystickPort = 1;

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 4;
        
        //Strafe buttons
        public static final int leftStrafe = 5;
        public static final int rightStrafe = 6;

        //Intake buttons
        public static final int kIntakeCloseButtonIdx = 5;


        // CPR = counts per revoulution
        public static final double SRXMagEncoderCPR = 1024; //if am-3445
        public static final double wheelDiameter = 8; //for 8 inch wheel
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.5;

        public static final double kAutoDriveDiagonalSpeed = 0.25;
        public static final double kAutoDriveLeftCornerSpeed = 0.25;
        public static final double kAutoDriveRightCornerSpeed = 0.25;
        public static final double kAutoDriveRoundTurnSpeed = 0.25;
        public static final double kAutoDriveSidewaysSpeed = 0.25;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        );
    }

    // TODO: Confirm all ports match the actual intake motors (based on wiring)
    public static final class DriveConstants {
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

        public static final int kFrontLeftMotorPort = 8;
        public static final int kFrontRightMotorPort = 7;
        public static final int kRearLeftMotorPort = 13;
        public static final int kRearRightMotorPort = 1;
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

    // TODO: Map these ports to the actual intake motors (based on wiring)
    public static final class ElevatorConstants {
        public static final int kVerticalMotorPort = 5;
        public static final int kSlideMotorPort = 4;
        public static final int kEncoderChannelA = 8;
        public static final int kEncoderChannelB = 9;
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;
        public static final double kP = 3;
        public static final double kI = 0;
        public static final double kD = 0.8;

        public static final double kRaisedPosition = 1.2;
        public static final double kLoweredPosition = 0;
        public static final double kJoystickMaxSpeed = 0.5;
    }

    // TODO: Map these ports to the actual intake motors (based on wiring)
    public static final class IntakeConstants {
        public static final int kLeftMotorPort = 15;
        public static final int kRightMotorPort = 9;
        public static final double kOpenSpeed = -1;
        public static final double kCloseSpeed = 1;
    }
}