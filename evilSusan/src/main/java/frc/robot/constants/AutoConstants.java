package frc.robot.constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
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

    public static final Set<Integer> driveForwardDepositTargetList = new HashSet<>(Arrays.asList(21, 10, 20));

// Constraint for the motion profilied robot angle controller
public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
);
}
