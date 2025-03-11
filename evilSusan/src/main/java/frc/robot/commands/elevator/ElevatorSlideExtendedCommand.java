package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;


public class ElevatorSlideExtendedCommand extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorSlideExtendedCommand(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    public ElevatorSlideExtendedCommand(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorSlideExtendedCommand started");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setSlidePosition(ElevatorConstants.kExtendedPosition);
        System.out.println("Exec ElevatorSlideExtendedCommand: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopSlideMotors();
        System.out.println("ElevatorSlideExtendedCommand ended");
    }
}
