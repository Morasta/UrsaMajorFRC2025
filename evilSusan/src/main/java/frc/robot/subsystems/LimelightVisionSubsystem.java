package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.Constants.LimelightVisionConstants;
import frc.robot.Constants.LimelightVisionConstants.LimelightCamera;
//import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.lib.VisionHelpers;
import frc.robot.lib.LimelightHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
// import limelight.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightVisionSubsystem extends SubsystemBase{
public static AprilTagFieldLayout fieldLayout;

// public final Limelight limelight = new Limelight("limelight-evlsusn");

public LimelightVisionSubsystem() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME);
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

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

    public double getXValue () {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("tx").getDouble(0.0);
    }

    public double getYValue () {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("ty").getDouble(0.0);
    }

    public double getAreaValue () {
        return NetworkTableInstance.getDefault().getTable(LimelightCamera.CAMERA_NAME).getEntry("ta").getDouble(0.0);
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

    @Override
    public void periodic() {
        if (fieldLayout != null) {
            //getting the pose of AprilTag with ID 1
            int tagID = 1;
            Pose3d tagPose = fieldLayout.getTagPose(tagID).orElse(null);
        }
    }
}
