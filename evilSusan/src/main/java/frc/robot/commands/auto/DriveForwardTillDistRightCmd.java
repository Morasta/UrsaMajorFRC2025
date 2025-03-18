package frc.robot.commands.auto;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.utils.CameraPositions;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Date;

public class DriveForwardTillDistRightCmd extends Command {
    private final DriveTrain driveSubsystem;
    private final LimelightVisionSubsystem visionSubsystem;
    private final double distance;
    private double speed = 0.3;
    private int notFoundCount;
    
    CameraPositions currentRobotPosition = new CameraPositions();
    private boolean closeEnoughToTarget = false;

    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    // public DriveForwardTillDistRightCmd(DriveTrain driveTrain, double distance) {
    //     this(driveTrain, limelightVision, distance, 1.0);
    // }

    public DriveForwardTillDistRightCmd(DriveTrain driveTrain, LimelightVisionSubsystem limelightVision, double distance, double speed) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.visionSubsystem = limelightVision;
        this.distance = 1; //TODO: Fix me
        this.speed = speed;
        //* this.distance = DriveTrain.getEncoderMeters() + distance; */
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
        this.notFoundCount = 0;
        
        currentRobotPosition = visionSubsystem.getCurrentPosition();
        //driveSubsystem.setMaxOutput(0.3);
    }

    @Override
    public void execute() {
        printStatus("executed");
        currentRobotPosition = visionSubsystem.getCurrentPosition();
        closeEnoughToTarget = Math.abs(AutoConstants.targetArea - visionSubsystem.getAreaValue()) <= AutoConstants.targetAreaGoalTolerance;

        if (!visionSubsystem.targetIsVisible())
            notFoundCount += 1;
        else 
            notFoundCount = 0;
        
        //set all motors at forward speed
        driveSubsystem.setMotors(speed, speed);
    }

    @Override
    public void end(boolean interrupted) {
        printStatus(" finished");

        driveSubsystem.setMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        printStatus("isfinished? " + !visionSubsystem.targetIsVisible() + " | " + closeEnoughToTarget);
        return notFoundCount >= AutoConstants.maxAprilTagNotFoundCount || closeEnoughToTarget;
    }
}
