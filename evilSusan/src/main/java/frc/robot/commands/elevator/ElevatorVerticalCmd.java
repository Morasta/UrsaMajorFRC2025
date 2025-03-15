package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;


public class ElevatorVerticalCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorVerticalCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    //TODO: Re-add this when encoders are implemented
    /*public ElevatorVerticalCmd(ElevatorSubsystem elevatorSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }*/

    @Override
    public void initialize() {
        System.out.println("ElevatorVerticalCmd started");
        elevatorSubsystem.setMotorBrakeMode(NeutralMode.Coast);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setVerticalMotor(speed);
        System.out.println("Exec ElevatorVerticalCmd: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        //elevatorSubsystem.stopVerticalMotors();
        elevatorSubsystem.setVerticalMotor(-0.10);
        elevatorSubsystem.setMotorBrakeMode(NeutralMode.Brake);
        System.out.println("ElevatorVerticalCmd ended");
    }
}
