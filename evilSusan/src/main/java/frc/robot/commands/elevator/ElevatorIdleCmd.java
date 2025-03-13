package frc.robot.commands.elevator;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;

public class ElevatorIdleCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorIdleCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
        SmartDashboard.putNumber("ElevatorIdleSpeed", speed);
    }

    public ElevatorIdleCmd(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
        SmartDashboard.putNumber("ElevatorIdleSpeed", speed);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorIdleCmd started");
    }

    @Override
    public void execute() {
        double idleSpeed = SmartDashboard.getNumber("ElevatorIdleSpeed", speed);
        elevatorSubsystem.setVerticalMotor(idleSpeed);
        System.out.println("Exec ElevatorIdleCmd: " + idleSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopVerticalMotors();
        System.out.println("ElevatorIdleCmd ended");
    }
}
