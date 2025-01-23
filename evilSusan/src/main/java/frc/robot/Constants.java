package frc.robot;

import java.lang.reflect.Array;

public final class Constants {
    // Constant definitions for controller mapping used by the "operator" or driver
    // This should map to the controller classes we'll define later
    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 3;
    } 

    public static final class DriveConstants {
        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 0.5;

        public static final int kFrontLeftMotorPort = 0;
        public static final int kFrontRightMotorPort = 1;
        public static final int kRearLeftMotorPort = 2;
        public static final int kRearRightMotorPort = 3;

        public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kRearLeftEncoderPorts = new int[] {2, 3};
        public static final int[] kFrontRightEncoderPorts = new int[] {4, 5};
        public static final int[] kRearRightEncoderPorts = new int[] {6, 7};

        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kRearLeftEncoderReversed = true;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kRearRightEncoderReversed = true;
        
        // TODO: do the math for this, such as: (kWheelDiameterMeters * Math.PI) / kEncoderCPR;
        // Assumes the encoders are directly mounted on the wheel shafts
        public static final double kEncoderDistancePerPulse = 1.0;
    }
}
