package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants.TargetTagsCoralStation.Red;
import frc.robot.constants.LimelightVisionConstants.LimelightCamera;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;

public class RotateTillTagFoundCmd extends Command {

    private final DriveTrain driveSubsystem;
    private final LimelightVisionSubsystem visionSubsystem;
    private final double distance;

    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public RotateTillTagFoundCmd(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, double distance, double speed) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.visionSubsystem = visionSubsystem;
        this.distance = 1; //TODO: Fix me
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
        driveSubsystem.feed(); 
        LimelightResults results = LimelightHelpers.getLatestResults(LimelightCamera.CAMERA_NAME);

        for (LimelightTarget_Fiducial target: results.targets_Fiducials) {
        System.out.println("In for loop. fID: " + target.fiducialID);
        switch ((int)target.fiducialID) {
            case Red.ReefTopRight:
                System.out.println("Reading AprilTag 20");
                visionSubsystem.setTarget(Red.ReefTopRight);
                //driveSubsystem.runOnce(() -> new DriveForwardCmd(driveSubsystem, AprilTagDists.ToReefStation));
                break;
            case Red.ReefRight:
                System.out.println("Reading AprilTag 21");
                visionSubsystem.setTarget(Red.ReefRight);
                //driveSubsystem.runOnce(() -> new DriveBackwardCmd(driveSubsystem, AprilTagDists.ToReefStation));
                break;
            case Red.ReefBottomRight:
                System.out.println("Reading AprilTag 22");
                visionSubsystem.setTarget(Red.ReefBottomRight);
                //driveSubsystem.runOnce(() -> new DriveRoundTurnCmd(driveSubsystem, AprilTagDists.ToReefStation));
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

    @Override
    public boolean isFinished() {
        return visionSubsystem.getTarget() != -1;
        //return Math.abs(lastValidTargetTY - targetDistance) < distanceTolerance && Math.abs(lastValidTargetAngle - targetAngle);
    }
}
