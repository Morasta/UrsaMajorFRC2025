package frc.robot.constants;

public class ElevatorConstants {
    public static enum ElevatorVerticalPositions {
        bottom
        , middle
        , top
    };
    public static final int kVerticalLeftMotorPort = 7;
    public static final int kVerticalRightMotorPort = 6;
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
    public static final double kJoystickMaxSpeed = 0.5;

    public static final double idleSpeed = -0.2;
    public static final double dropSpeed = 0.09;
    public static final double upSpeed = -1;
    public static final double autoSpeed = -0.10;
}
