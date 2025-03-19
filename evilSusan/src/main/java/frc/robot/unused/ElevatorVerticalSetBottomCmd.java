package frc.robot.unused;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.ElevatorVerticalPositions;

public class ElevatorVerticalSetBottomCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorVerticalSetBottomCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    public ElevatorVerticalSetBottomCmd(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorVerticalSetBottomCmd started");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setVerticalPosition(ElevatorConstants.kLoweredPosition);
        System.out.println("Exec ElevatorVerticalSetBottomCmd: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopVerticalMotors();
        System.out.println("ElevatorVerticalSetBottomCmd ended");
    }
}
