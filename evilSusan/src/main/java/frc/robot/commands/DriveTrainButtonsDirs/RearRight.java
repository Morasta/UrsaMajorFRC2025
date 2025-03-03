package frc.robot.commands.DriveTrainButtonsDirs;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Date;

public class RearRight extends Command {
    private final DriveTrain driveSubsystem;
    private final double distance;
    private Long startTime;
  
    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    public RearRight(DriveTrain driveTrain, double distance) {
        this(driveTrain, distance, 1.0);
    }

    public RearRight(DriveTrain driveTrain, double distance, double speed) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.distance = 1; //TODO: Fix me
        //* this.distance = DriveTrain.getEncoderMeters() + distance; */
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
        startTime=System.currentTimeMillis(); // in init
    }

    @Override
    public void execute() {
        printStatus("executed");
        //set all motors at forward speed
        //driveSubsystem.setMotors(0.3, 0.3);
        //TODO: drive back 2 motors (TalonSRX)
        driveSubsystem.setMotors(0, 0, 0, 1);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }
}