package frc.robot.lib;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.VisionHelperConstants;
import frc.robot.constants.RobotChassisConstants;
import frc.robot.constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.subsystems.LimelightVisionSubsystem;

public class VisionHelpers {

    //TODO: change all instances of SwerveChassis to Mecanum

    public static double getDistanceBetweenCenterCameraCoral(Double alpha, Double beta, double h, double d) {
        if (!alpha.isNaN() && beta.isNaN()) {
            return calculateDistanceUsingAlpha(alpha, h, d);
        } else if (alpha.isNaN() && !beta.isNaN()) {
            return calculateDistanceUsingAlpha(beta, h, d);
        } else if (!alpha.isNaN() && !beta.isNaN()) {
            if(Math.abs(alpha)>beta) {
                return calculateDistanceUsingBeta(beta, h, d);
            } else {
                return calculateDistanceUsingAlpha(alpha, h, d);
            }
        }
        return 0;
    }

    public static double calculateDistanceUsingAlpha(Double alpha, double  h, double d) {
        double x = (h-Math.tan(90 - Math.abs(alpha))*d/2.0)/Math.tan(90 - Math.abs(alpha));
        return x;
    }

    public static double calculateDistanceUsingBeta(Double beta, double h, double d) {
        double x = d/2.0 - h*Math.tan(Math.abs(beta));
        return x;
    }

    public static double calculateTheta (double d, Double alpha, Double beta, double h) {
        double c = (Math.sin(90-beta) * d)/Math.sin(Math.abs(alpha) * Math.abs(beta));
        double x =(Math.sin(90 - Math.abs(alpha)) * c);
        double l = (x / Math.tan(90 -Math.abs(alpha)));
        double y = (-(1 * x)/ h) + ((d * x) / (2 * h));
        double z = 1 - (x / h);
        double totalLength = y/z;
        return Math.atan(x / totalLength);
    }

    /*
     * Get the Pose of a specific AprilTag by its ID.
     * @param tagID the ID of the AprilTag.
     * @return the Pose of the AprilTag as a Pose3d, or null if the tag is not found.
     */

    public static Pose3d getTagPose(int tagID) {
        if (LimelightVisionSubsystem.fieldLayout != null) {
            Optional<Pose3d> tagPose = LimelightVisionSubsystem.fieldLayout.getTagPose(tagID);
            if (tagPose.isPresent()) {
                return tagPose.get();
            } else {
                DriverStation.reportWarning("AprilTag " + tagID + " not found in the field Layout.", false);
            }
        }
        return null;
    }

    /*
     *TODO: instructions unclear. rewrite to make sense
     * Coral Station side are LOWER and HIGHER. for BLUE the LOWER is on the RIGHT and HIGHER is on the LEFT.
     * For RED it's oppisite - LOWER on the LEFT and HIGHER on the RIGHT from Drivers POV
     * 
     * Reef sides are:
     * 1 - facing the driver on correspoonding side (tags 18 for BLUE and 7 for RED)
     * 2 - RED - COUNTERCLOCKWISE from it from Drivers POV (tags 8 for RED)
     * same for sides 3, 4, 5, 6.
     * BLUE - CLOCKWISE from it from Driver POV (tags 19 for BLUE)
     */

    public static void createHashMapOfTags() {
        //RED Tags
        RobotPoseConstants.visionRobotPoses.put("TagRedCoralLOW", getTagPose(1).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedCoralHIGH", getTagPose(2).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef1", getTagPose(7).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef2", getTagPose(8).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef3", getTagPose(9).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef4", getTagPose(10).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef5", getTagPose(11).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef6", getTagPose(6).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedBarge", getTagPose(5).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedProcessor", getTagPose(3).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedBargeByProcessor", getTagPose(4).toPose2d());
        //BLUE Tags
        RobotPoseConstants.visionRobotPoses.put("TagBlueCoralLOW", getTagPose(12).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueCoralHIGH", getTagPose(13).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueReef1", getTagPose(18).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueReef2", getTagPose(19).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueReef3", getTagPose(20).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueReef4", getTagPose(21).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueReef5", getTagPose(22).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueReef6", getTagPose(17).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueBarge", getTagPose(14).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueProcessor", getTagPose(16).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBlueBargeByProcessor", getTagPose(15).toPose2d());
        
    }

    /*
     * Move the Pose in a Pose-centric (relative) way by X and Y without changing rotation.
     * ex: if Pose points LEFT (Rotation 90 degrees), the move of 0,1 moves the Y of the Pose by +1.
     * The rotation od the Pose will not change
     */

    public static Pose2d movePoseXY(Pose2d pose, double x, double y) {
        return pose.transformBy(new Transform2d(x, y, Rotation2d.kZero));
    }

    public static void addRobotPosesForCoralPlacement() {
        List<String> keys = new ArrayList<>();
        for(String k : RobotPoseConstants.visionRobotPoses.keySet()) {
            keys.add(k);
        }
        for (String key :keys) {
            // Fill robot Poses
            if (key.contains("Reef") && key.contains("Tag")) {
                RobotPoseConstants.visionRobotPoses.put(
                    // Ex: RobotBlueReef1Left
                    "Robot" + key.substring(3,6) + "Reef" +
                         key.substring(10,11) + "Left",
                         movePoseXY(RobotPoseConstants.visionRobotPoses.get(key).plus(new Transform2d(0, 0, Rotation2d.k180deg)),
                         -(VisionHelperConstants.bumperWidth + (RobotChassisConstants.WHEEL_BASE / 2.0)),
                         VisionHelperConstants.distanceBetweenReefPoles / 2.0)
                );
                RobotPoseConstants.visionRobotPoses.put(
                    "Robot" + key.substring(3,6) + "Reef" + key.substring(10,11) + "Right",
                        movePoseXY(
                            RobotPoseConstants.visionRobotPoses.get(key).plus(new Transform2d(0, 0, Rotation2d.k180deg)),
                            -(VisionHelperConstants.bumperWidth + (RobotChassisConstants.WHEEL_BASE / 2.0)),
                            VisionHelperConstants.distanceBetweenReefPoles / 2.0
                        )
                );
            } else if ( (key.contains("Coral") || key.contains("Processor")) && key.contains("Tag")) {
                RobotPoseConstants.visionRobotPoses.put(
                    "Robot" + key.substring(3),
                    movePoseXY(RobotPoseConstants.visionRobotPoses.get(key).plus(new Transform2d(0,0, Rotation2d.k180deg)),
                    -(VisionHelperConstants.bumperWidth + (RobotChassisConstants.WHEEL_BASE / 2.0)), 0)
                );
            }
        }
        System.out.println(RobotPoseConstants.visionRobotPoses);
    }

    public static boolean isFieldLayoutValid() {
        return LimelightVisionSubsystem.fieldLayout != null;
    }
    
}
