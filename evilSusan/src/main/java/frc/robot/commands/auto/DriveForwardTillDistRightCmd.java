package frc.robot.commands.auto;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.utils.CameraPositions;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Date;

public class DriveForwardTillDistRightCmd extends Command {
    private final DriveTrain driveSubsystem;
    private final LimelightVisionSubsystem visionSubsystem;
    private final double distance;
    private long startTime;
    private long endTime;
    private final long executionTime = 1000;
    CameraPositions currentRobotPosition = new CameraPositions();
  
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
        //* this.distance = DriveTrain.getEncoderMeters() + distance; */
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
        startTime = System.currentTimeMillis(); // in init
        endTime = startTime + executionTime;
        currentRobotPosition = visionSubsystem.getCurrentPosition();
        driveSubsystem.setMaxOutput(0.3);
    }

    @Override
    public void execute() {
        currentRobotPosition = visionSubsystem.getCurrentPosition();
        printStatus("executed");
        //set all motors at forward speed
        if (Math.abs(AutoConstants.targetArea - visionSubsystem.getAreaValue()) > AutoConstants.targetAreaGoalTolerance) {
            driveSubsystem.setMotors(0.3, 0.3);
        } else {
            return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
        driveSubsystem.setMaxOutput(0);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }
}
