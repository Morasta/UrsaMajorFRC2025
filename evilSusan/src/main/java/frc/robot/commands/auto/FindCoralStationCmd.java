package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.AprilTagDists;
import frc.robot.Constants.AutoConstants.TargetTagsCoralStation.Red;
import frc.robot.commands.drive.DriveBackwardCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.lib.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;
import limelight.networktables.LimelightSettings.LEDMode;

public class FindCoralStationCmd extends Command {

    private final DriveTrain driveSubsystem;
    private final LimelightVisionSubsystem visionSubsystem;
    //private final double distance;

    // limelight.pipelineSwitch(0);

    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public FindCoralStationCmd(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, double distance) {
        this(driveTrain, visionSubsystem, distance, 1.0);
    }

    public FindCoralStationCmd(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, double distance, double speed) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.visionSubsystem = visionSubsystem;
        //this.distance = distance; //TODO: Fix me
        //* this.distance = DriveTrain.getEncoderMeters() + distance; */
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }

    @Override
    public void execute() {  
        //printStatus("executed: " + LimelightHelpers.getTX("limelight"));

        LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        printStatus("executed: " + visionSubsystem.getTargetTX());
        
        //LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        //RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");

        //visionSubsystem.limelight.getSettings()
        //    .withLimelightLEDMode(LEDMode.PipelineControl)
        //    .withCameraOffset(Pose3d.kZero)
        //    .save();

        //printStatus("executed" + visionSubsystem.limelight.getLatestResults().toString());

        /*
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
        */
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
