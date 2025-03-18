package frc.robot.commands.elevator;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.constants.ElevatorConstants.ElevatorVerticalPositions;

public class ElevatorIdleCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorIdleCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    public ElevatorIdleCmd(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorIdleCmd started");
    }

    @Override
    public void execute() {
        //double rawIdleSpeed = SmartDashboard.getNumber("ElevatorIdleSpeed", speed);
        
        // Apply limits (adjust these values based on your system)
        //double limitedSpeed = Math.max(-0.5, Math.min(0.5, rawIdleSpeed));
        elevatorSubsystem.setVerticalMotor(speed);
        //System.out.println("Exec ElevatorIdleCmd: " + limitedSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopVerticalMotors();
        System.out.println("ElevatorIdleCmd ended");
    }
}
