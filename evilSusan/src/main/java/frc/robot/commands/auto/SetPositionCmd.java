package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.utils.CameraPositions;

public class SetPositionCmd extends Command{

    private final LimelightVisionSubsystem visionSubsystem;
    private final DriveTrain driveSubsystem;
    private boolean targetFound = false;
    CameraPositions currentRobotPosition = new CameraPositions();

    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public SetPositionCmd(LimelightVisionSubsystem limelightVision, DriveTrain driveTrain, double distance) {
        printStatus("Created");
        this.visionSubsystem = limelightVision;
        this.driveSubsystem= driveTrain;
        addRequirements(limelightVision);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
        currentRobotPosition = visionSubsystem.getCurrentPosition();
    }

    @Override
    public void execute() {
        printStatus("Execute SetPositionCmd");
        currentRobotPosition = visionSubsystem.getCurrentPosition();

        if (currentRobotPosition.tx - AutoConstants.targetTxPosition > AutoConstants.targetCamTolerance) {
            //Crabwalk Right
            driveSubsystem.setMotors(0.3, -0.3, -0.3, 0.3);
        } else if ((currentRobotPosition.tx - AutoConstants.targetTxPosition) * -1 < AutoConstants.targetCamTolerance) {
            //Crabwalk Left
            driveSubsystem.setMotors(-0.3, 0.3, 0.3, -0.3);
        } else {
            this.targetFound = true;
        }

        visionSubsystem.updateCurrentPosition();
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("ended");
        driveSubsystem.setMotors(0, 0);
        visionSubsystem.updateCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        return this.targetFound;
    }
}