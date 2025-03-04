package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;


public class ElevatorSlideRetractedCommand extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorSlideRetractedCommand(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    public ElevatorSlideRetractedCommand(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorSlideRetractedCommand started");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setVerticalPosition(ElevatorConstants.kRetractedPosition);
        System.out.println("Exec ElevatorSlideRetractedCommand: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotors();
        System.out.println("ElevatorSlideRetractedCommand ended");
    }
}
