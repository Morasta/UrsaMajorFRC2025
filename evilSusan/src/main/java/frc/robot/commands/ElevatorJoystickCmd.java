package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorJoystickCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
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
        elevatorSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotor(0);
        System.out.println("ElevatorJoystickCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
