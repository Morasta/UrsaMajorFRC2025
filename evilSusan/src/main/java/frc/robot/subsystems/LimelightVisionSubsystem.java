package frc.robot.subsystems;

import frc.robot.lib.LimelightHelpers;
import frc.robot.constants.LimelightVisionConstants.LimelightCamera;
//import frc.robot.Constants.EnabledSubsystems;
import frc.robot.lib.VisionHelpers;
import frc.robot.utils.CameraPositions;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightVisionSubsystem extends SubsystemBase{
    public static AprilTagFieldLayout fieldLayout;
    private static int targetID = -1;

    private final CameraPositions camPos = new CameraPositions();
    
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
    } 
    
    public void updateCurrentPosition() {
        this.camPos.tx = getXValue();
        this.camPos.ty = getYValue();
        this.camPos.ta = getAreaValue();
        this.camPos.tv = targetIsVisible();
    }

    public CameraPositions getCurrentPosition() {
        return this.camPos;
    }
    
    public double getXValue() {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("tx").getDouble(0.0);
    }

    public double getCamPosInTargetSpaceValue() {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("t6s_ts").getDouble(0.0);
    }

    public double getRobotPosinTargetSpaceValue() {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("t6r_ts").getDouble(0.0);
    }
    
    public double getYValue() {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("ty").getDouble(0.0);
    }
    
    public double getAreaValue() {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("ta").getDouble(0.0);
    }

    public boolean targetIsVisible() {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("tv").getDouble(0.0) == 1;
    }

    /* Set led mode of the limelight
     *   0 use the LED Mode set in the current pipeline
     *   1 force off
     *   2 force blink
     *   3 force on
    */
    public void setLEDMode(int mode) {
        NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("ledMode").setNumber(mode);
    }

    /* Set camera mode of the limelight
     *   0 Vision processor
     *   1 Driver Camera (Increases exposure, disables vision processing)
    */
    public void setCamMode(int mode) {
        NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("camMode").setNumber(mode);
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
    
    public boolean isAprilTagVisable(String cameraName) {
        return LimelightHelpers.getTV(cameraName);
    }

    public void setTarget(int targetID) {
        this.targetID = targetID;
    }

    public int getTarget() {
        return this.targetID;
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
