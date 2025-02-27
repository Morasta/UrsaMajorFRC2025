package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
//import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.lib.VisionHelpers;
import frc.robot.lib.LimelightHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import limelight.Limelight;

public class LimelightVisionSubsystem extends SubsystemBase{
    
    public static AprilTagFieldLayout fieldLayout;
    public static final Limelight limelight = new Limelight("limelight");
    
    public LimelightVisionSubsystem() {
        /*if(!EnabledSubsystems.ll) {
            return;
        }*/
        
        try {
            //TODO: make sure this is up to date on competition days
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load Apriltag Field Layout: " + e.getMessage(), true);
            fieldLayout = null;
        }
        
        //load Pose2d of AprilTags
        VisionHelpers.createHashMapOfTags();
        // load Pose2d of robot to interact with game elements
        VisionHelpers.addRobotPosesForCoralPlacement();
        
        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("limelight", 0);
        //LimelightHelpers.setLEDMode_PipelineControl("limelight");
        //LimelightHelpers.setStreamMode_Standard("limelight");
        LimelightHelpers.setLimelightNTDouble("limelight", "myfoo", 1.9);
    }   
    
    public Pose2d getRobotAprilTagPose() {
        return null;
    }
    
    /* 
    public Pose2d getKnownPose(String poseName) {
        if(RobotPoseConstants.visionRobotPoses.containsKey(poseName)) {
            return RobotPoseConstants.visionRobotPoses.get(poseName);
        } else {
            return null;
        }
    }
    */

    public double getTargetAngle() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }

    public double getTargetTY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    }

    public double getTargetTX() {
        System.out.println("getTx: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
        System.out.println("getFoo: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("myfoo").getDouble(0));
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }

    public double[] get3DPose() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[6]);
    }

    public boolean isAprilTagVisable(String cameraName) {
        return LimelightHelpers.getTV(cameraName);
    }
    
    @Override
    public void periodic() {
        if (fieldLayout != null) {
            //getting the pose of AprilTag with ID 1
            int tagID = 1;
            Pose3d tagPose = fieldLayout.getTagPose(tagID).orElse(null);
        }
    }
}
