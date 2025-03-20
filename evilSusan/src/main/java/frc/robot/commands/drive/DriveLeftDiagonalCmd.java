package frc.robot.commands.drive;

import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveLeftDiagonalCmd extends Command {
        private final DriveTrain driveSubsystem;
    private final double distance;
  
    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public DriveLeftDiagonalCmd(DriveTrain driveTrain, double distance) {
        this(driveTrain, distance, 1.0);
    }

    public DriveLeftDiagonalCmd(DriveTrain driveTrain, double distance, double speed) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.distance = 1; 
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
        printStatus("executed");
        driveSubsystem.setMotors(0, 0.3, 0.3, 0);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }
}
