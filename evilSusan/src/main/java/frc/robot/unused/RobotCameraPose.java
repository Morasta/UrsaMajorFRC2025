package frc.robot.unused;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DriveTrain;
import frc.robot.constants.AutoConstants.AprilTagDists;
import frc.robot.constants.AutoConstants.TargetTagsCoralStation.Red;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.commands.drive.DriveBackwardCmd;

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