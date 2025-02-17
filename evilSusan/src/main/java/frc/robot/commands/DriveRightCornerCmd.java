package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveRightCornerCmd extends Command {
    private final DriveTrain driveSubsystem;
    private final double distance;
  
    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public DriveRightCornerCmd(DriveTrain driveTrain, double distance) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.distance = 1; //TODO: Fix me
        //* this.distance = DriveTrain.getEncoderMeters() + distance; */
        // addRequirements(DriveTrain);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }

    @Override
    public void execute() {
        printStatus("executed");
        driveSubsystem.setMotors(1, 0, 1, 0);
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }
}
