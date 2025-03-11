package frc.robot.commands.elevator;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ElevatorConstants.ElevatorSlidePositions;


public class ElevatorSlideCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorSlideCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    public ElevatorSlideCmd(ElevatorSubsystem elevatorSubsystem, ElevatorSlidePositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    
    @Override
    public void initialize() {
        System.out.println("ElevatorSlideCmd started");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setSlideMotor(speed);
        System.out.println("Exec ElevatorSlideCmd: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopSlideMotors();
        System.out.println("ElevatorSlideCmd ended");
    }
}
