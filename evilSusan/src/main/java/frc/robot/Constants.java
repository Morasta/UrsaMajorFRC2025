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

        public static final int kFrontLeftMotorPort = 1;
        public static final int kFrontRightMotorPort = 1;
        public static final int kRearLeftMotorPort = 1;
        public static final int kRearRightMotorPort = 1;
        public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kEncoderDistancePerPulset = new int[] {0, 1};
        public static final int[] kFrontRightEncoderPorts = new int[] {0, 1};
        public static final boolean kFrontRightEncoderReversed = true;
        public static final boolean kRearRightEncoderReversed = true; 
        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kRearLeftEncoderReversed = false;
        public static final int[] kRearLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kRearRightEncoderPorts = new int[] {0, 1};
        public static final double kEncoderDistancePerPulse = 1.0;
    }
}
