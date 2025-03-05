package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.utils.CameraPositions;

public class SetPositionCmd extends Command{

    private final LimelightVisionSubsystem visionSubsystem;
    private final DriveTrain driveSubsystem;
    private double speed = 1;
    private boolean targetFound = false;
    private int notFoundCount;
    CameraPositions currentRobotPosition = new CameraPositions();

    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public SetPositionCmd(LimelightVisionSubsystem limelightVision, DriveTrain driveTrain, double distance, double speed) {
        printStatus("Created");
        this.visionSubsystem = limelightVision;
        this.driveSubsystem= driveTrain;
        this.speed = speed;

        addRequirements(limelightVision);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
        currentRobotPosition = visionSubsystem.getCurrentPosition();
        this.notFoundCount = 0;
    }

    @Override
    public void execute() {
        printStatus("Execute SetPositionCmd");
        currentRobotPosition = visionSubsystem.getCurrentPosition();

        if (visionSubsystem.targetIsVisible() && currentRobotPosition.tx - AutoConstants.targetTxPosition > AutoConstants.targetCamTolerance) {
            //Crabwalk Right
            driveSubsystem.setMotors(speed, -speed, -speed, speed);
        } else if (visionSubsystem.targetIsVisible() && ((currentRobotPosition.tx - AutoConstants.targetTxPosition) * -1) < AutoConstants.targetCamTolerance) {
            //Crabwalk Left
            driveSubsystem.setMotors(-speed, speed, speed, -speed);
        } else {
            this.targetFound = true;
        }

        if (!visionSubsystem.targetIsVisible())
            notFoundCount += 1;
        else 
            notFoundCount = 0;

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
        return notFoundCount >= AutoConstants.maxAprilTagNotFoundCount || this.targetFound;
    }
}