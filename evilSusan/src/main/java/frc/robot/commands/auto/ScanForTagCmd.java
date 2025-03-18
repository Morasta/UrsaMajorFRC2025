package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightVisionConstants.LimelightCamera;
import frc.robot.commands.drive.DriveRightSidewaysCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;

public class ScanForTagCmd extends Command {

    private final DriveTrain driveSubsystem;
    private final LimelightVisionSubsystem visionSubsystem;
    private final double distance;
    private final int targetID;

    private void printStatus(String stateStatus) {
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public ScanForTagCmd(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, double distance,
            double speed, int targetID) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.visionSubsystem = visionSubsystem;
        this.distance = 1; // TODO: Fix me
        this.targetID = targetID;
        // * this.distance = DriveTrain.getEncoderMeters() + distance; */
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
        // LimelightResults results =
        // LimelightHelpers.getLatestResults(LimelightCamera.CAMERA_NAME);

        // rotate until target is identified
        while (!targetFound()) {
            // driveSubsystem.setMotors(0.3, 0.3, -0.3, -0.3);

            /*
             * Note: calling a command from another command might not work
             * if not, you can call setMotors() directly:
             * 
             * driveSubsystem.setMotors(0.3, 0.3, -0.3, -0.3);
             * 
             * However, you cannot call withTimeout() in this case, so we will need to
             * figure out
             * how we want to limit the rotation in that case.
             */
            driveSubsystem.runOnce(() -> new DriveRoundTurnCmd(driveSubsystem, 1, 0.3).withTimeout(0.5)); // TODO:
                                                                                                          // adjust
                                                                                                          // timeout as
                                                                                                          // needed
        }
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(lastValidTargetTY - targetDistance) < distanceTolerance &&
        // Math.abs(lastValidTargetAngle - targetAngle);
        return false;
    }

    // Grab the results from the camera, and check if the identified tag's ID
    // matches the
    // target tag ID
    public boolean targetFound() {
        LimelightResults results = LimelightHelpers.getLatestResults(LimelightCamera.CAMERA_NAME);

        LimelightTarget_Fiducial identifedTag = results.targets_Fiducials[0];

        if (identifedTag.fiducialID == targetID) {
            return true;
        }
        return false;
    }
}
