package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardCmd extends CommandBase {
    private final DriveTrain driveSubsystem;
    private final double distance;
  
    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public DriveForwardCmd(DriveTrain DriveTrain, double distance) {
        printStatus("Created");
        this.driveSubsystem = DriveTrain;
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
        //DriveTrain.setMotors(DriveConstants.kAutoDriveForwardSpeed, DriveConstants.kAutoDriveForwardSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("end");
        //DriveTrain.setMotors(0, 0);
        System.out.println(this.getClass().getSimpleName() + " executed");
    }
}
