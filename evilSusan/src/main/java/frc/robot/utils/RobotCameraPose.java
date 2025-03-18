package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Barcode;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DriveTrain;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.AutoConstants.AprilTagDists;
import frc.robot.constants.AutoConstants.TargetTagsCoralStation.Blue;
import frc.robot.constants.AutoConstants.TargetTagsCoralStation.Red;
import frc.robot.constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.commands.auto.FindAprilTagCmd;
import frc.robot.commands.drive.DriveBackwardCmd;
import frc.robot.subsystems.DriveTrain;

public class RobotCameraPose extends SubsystemBase {

    private final DriveTrain m_robotDrive = new DriveTrain();

    public RobotCameraPose() {

        LimelightResults results = LimelightHelpers.getLatestResults("");

        for (LimelightTarget_Fiducial target: results.targets_Fiducials) {
        switch ((int)target.fiducialID) {
            case Red.ReefTopRight:
            m_robotDrive.runOnce(() -> new DriveForwardCmd(m_robotDrive, AprilTagDists.ToReefStation));
            break;
            case Red.ReefRight:
            m_robotDrive.runOnce(() -> new DriveBackwardCmd(m_robotDrive, AprilTagDists.ToReefStation));
            break;
            case Red.ReefBottomRight:
            m_robotDrive.runOnce(() -> new DriveRoundTurnCmd(m_robotDrive, AprilTagDists.ToReefStation));
            break;
        }
    }




    }
}