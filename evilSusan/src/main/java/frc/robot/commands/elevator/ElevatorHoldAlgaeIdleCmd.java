package frc.robot.commands.elevator;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;

public class ElevatorHoldAlgaeIdleCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final boolean holdingAlgae;

    public ElevatorHoldAlgaeIdleCmd(ElevatorSubsystem elevatorSubsystem, boolean holdingAlgae) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.holdingAlgae = holdingAlgae;
        addRequirements(elevatorSubsystem);
    }

    public ElevatorHoldAlgaeIdleCmd(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, boolean holdingAlgae) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.holdingAlgae = holdingAlgae;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorHoldAlgaeIdleCmd started");
    }

    @Override
    public void execute() {
        //double rawIdleSpeed = SmartDashboard.getNumber("ElevatorIdleSpeed", speed);
        
        // Apply limits (adjust these values based on your system)
        //double limitedSpeed = Math.max(-0.5, Math.min(0.5, rawIdleSpeed));
        elevatorSubsystem.setIdleElevator(holdingAlgae);
        System.out.println("Exec ElevatorHoldAlgaeIdleCmd");
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopVerticalMotors();
        System.out.println("ElevatorHoldAlgaeIdleCmd ended");
    }
}
