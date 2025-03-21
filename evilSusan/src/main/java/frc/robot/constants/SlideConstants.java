package frc.robot.constants;

public class SlideConstants {
    public static enum SlidePositions {
        back
        , middle
        , front
    }

    public static final int kSlideMotorPort = 4;
    public static final double kExtendedPosition = 1.2;
    public static final double kRetractedPosition = 0;
    public static final double slideOutSpeed = 0.5;
    public static final double slideInSpeed = -0.5;
}
