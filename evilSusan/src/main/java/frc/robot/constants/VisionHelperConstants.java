package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class VisionHelperConstants {
public static final double distanceBetweenReefPoles = Units.inchesToMeters(12.94);
public static final double bumperWidth = Units.inchesToMeters(2.5);
public static class RobotPoseConstants {
        public static Map<String, Pose2d> visionRobotPoses = new HashMap<String, Pose2d>();
         public static Map<Integer, String> tagNumberToKey = new HashMap<Integer, String>();
    }
}
