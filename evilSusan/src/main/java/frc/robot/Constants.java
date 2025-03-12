package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;



public final class Constants {
    // Constant definitions for controller mapping used by the "operator" or driver
    // This should map to the controller classes we'll define later

    public static final class Physics {
        public static final double GRAVITY_COMPENSATION = 1.0;
    }
    
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

        //TODO: figure out what num this should be
        public static final double targetTxPosition = 20;
        public static final double targetArea = 6;
        public static final double targetAreaGoalTolerance = 1.5;
        public static final double targetCamTolerance = 1.5;
        public static final int maxAprilTagNotFoundCount = 10;

        public static final class AprilTagDists {
            //TODO: fix to correct dists
            public static final double ToReefStation = 3.0;
            public static final double ToCoralStation = 5.0;
            public static final double ToBarge = 6.0;
        }

        public static final class TargetTagsCoralStation {
            public static final class Red {
                public static final int ReefTopLeft = 19;
                public static final int ReefTopRight = 20;
                public static final int ReefLeft = 18;
                public static final int ReefRight = 21;
                public static final int ReefBottomLeft = 17;
                public static final int ReefBottomRight = 22;
            }
            public static final class Blue {
                public static final int ReefTopLeft = 9;
                public static final int ReefTopRight = 8;
                public static final int ReefLeft = 10;
                public static final int ReefRight = 7;
                public static final int ReefBottomLeft = 11;
                public static final int ReefBottom = 6;
            }
        }

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        );
    }

    public static final class RobotChassis {
        //TODO: change to correct size
        public static final double WHEEL_BASE = Meters.convertFrom(18.00, Inches);
        // CPR = counts per revoulution
        public static final double SRXMagEncoderCPR = 1024; //if am-3445
        public static final double wheelDiameter = 8; //for 8 inch wheel
    }

    public static final class VisionHelperConstants {
        public static final double distanceBetweenReefPoles = Units.inchesToMeters(12.94);
        public static final double bumperWidth = Units.inchesToMeters(2.5);
        public static class RobotPoseConstants {
            public static Map<String, Pose2d> visionRobotPoses = new HashMap<String, Pose2d>();
            public static Map<Integer, String> tagNumberToKey = new HashMap<Integer, String>();
        }
    }

    public static final class LimelightVisionConstants {
        public static final class LimelightCamera {
            public static String CAMERA_NAME = "limelight";
        }
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

    // TODO: Map these ports to the actual intake motors (based on wiring)
    public static final class ElevatorConstants {
        public static enum ElevatorVerticalPositions {
            bottom
            , middle
            , top
        };
        public static enum ElevatorSlidePositions {
            back
            , middle
            , front
        };
        public static final int kVerticalLeftMotorPort = 7;
        public static final int kVerticalRightMotorPort = 6;
        public static final int kSlideMotorPort = 4;
        public static final int kEncoderChannelA = 16;
        public static final int kEncoderChannelB = 17;
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;
        public static final double kP = 3;
        public static final double kI = 0;
        public static final double kD = 0.8;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kRaisedPosition = 1.2;
        public static final double kLoweredPosition = 0;
        public static final double kExtendedPosition = 1.2;
        public static final double kRetractedPosition = 0;
        public static final double kJoystickMaxSpeed = 0.5;
    }

    // TODO: Map these ports to the actual intake motors (based on wiring)
    public static final class IntakeConstants {
        public static final int kTopMotorPort = 8; //TODO: change to top
        public static final int kBottomMotorPort = 10;// and bottom
        public static final double kOpenSpeed = -0.5;
        public static final double kCloseSpeed = 0.5;
        public static final int kLeftEncoderA = 10;
        public static final int kLeftEncoderB = 11;
        public static final int kRightEncoderA = 12;
        public static final int kRightEncoderB = 13;
        public static final double kCoralDepositSpeed = kCloseSpeed;
        public static final double kCoralIntakeSpeed = kOpenSpeed;
    }
}
