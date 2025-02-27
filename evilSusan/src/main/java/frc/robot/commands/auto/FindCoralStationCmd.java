package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.AprilTagDists;
import frc.robot.Constants.AutoConstants.TargetTagsCoralStation.Red;
import frc.robot.commands.drive.DriveBackwardCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DriveTrain;

public class FindCoralStationCmd extends Command{

    private final DriveTrain driveSubsystem;
    private final double distance;

    // limelight.pipelineSwitch(0);



    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public FindCoralStationCmd(DriveTrain driveTrain, double distance) {
        this(driveTrain, distance, 1.0);
    }

    public FindCoralStationCmd(DriveTrain driveTrain, double distance, double speed) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.distance = 1; //TODO: Fix me
        //* this.distance = DriveTrain.getEncoderMeters() + distance; */
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }

    @Override
    public void execute() {            
        LimelightResults results = LimelightHelpers.getLatestResults("limelight-front");

        printStatus("executed" + results.toString());

        for (LimelightTarget_Fiducial target: results.targets_Fiducials) {
            System.out.println("In for loop. fID: " + target.fiducialID);
            switch ((int)target.fiducialID) {
                case Red.ReefTopRight:
                    System.out.println("Reading AprilTag 20");
                    driveSubsystem.runOnce(() -> new DriveForwardCmd(driveSubsystem, AprilTagDists.ToReefStation));
                    break;
                case Red.ReefRight:
                    System.out.println("Reading AprilTag 21");
                    driveSubsystem.runOnce(() -> new DriveBackwardCmd(driveSubsystem, AprilTagDists.ToReefStation));
                    break;
                case Red.ReefBottomRight:
                    System.out.println("Reading AprilTag 22");
                    driveSubsystem.runOnce(() -> new DriveRoundTurnCmd(driveSubsystem, AprilTagDists.ToReefStation));
                    break;
                default:
                    System.out.println("Default case");
                    break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }
}
