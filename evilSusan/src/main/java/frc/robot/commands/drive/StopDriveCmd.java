package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class StopDriveCmd extends Command{
    private final DriveTrain driveSubsystem;

    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public StopDriveCmd(DriveTrain driveTrain, double distance) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        printStatus("Stopped");
        //set all motors at forward speed
        driveSubsystem.setMotors(0, 0);
    }
}
