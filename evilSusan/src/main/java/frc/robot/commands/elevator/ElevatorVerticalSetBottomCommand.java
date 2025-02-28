package frc.robot.commands.elevator;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ElevatorVerticalPositions;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorVerticalCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorVerticalCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    public ElevatorVerticalCmd(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorJoystickCmd started!");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setVerticalPosition(kLoweredPosition);
        System.out.println("executing elevator vertical command, " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setVerticalMotor(0);
        System.out.println("ElevatorJoystickCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
