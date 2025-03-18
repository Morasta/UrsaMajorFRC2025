package frc.robot.commands.elevator;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorVerticalSetTopCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorVerticalSetTopCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorVerticalSetTopCmd started");
    }

    @Override
    public void execute() {
        System.out.println("Exec ElevatorVerticalSetTopCmd");
        elevatorSubsystem.setVerticalPosition(frc.robot.constants.ElevatorConstants.kRaisedPosition);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopVerticalMotors();
        System.out.println("ElevatorVerticalSetTopCmd ended");
    }
}
