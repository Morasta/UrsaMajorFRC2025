package frc.robot;

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
    }
}
