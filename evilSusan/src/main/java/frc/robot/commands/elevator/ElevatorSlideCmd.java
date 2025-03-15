package frc.robot.commands.elevator;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SlideSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ElevatorConstants.ElevatorSlidePositions;
import frc.robot.Constants.SlideConstants;


public class ElevatorSlideCmd extends Command{
    private final SlideSubsystem slideSubsystem;
    private final double speed;

    public ElevatorSlideCmd(SlideSubsystem slideSubsystem, double speed) {
        this.slideSubsystem = slideSubsystem;
        this.speed = speed;
        addRequirements(slideSubsystem);
    }

    public ElevatorSlideCmd(SlideSubsystem slideSubsystem, ElevatorSlidePositions targetPosition, double speed) {
        this.slideSubsystem = slideSubsystem;
        this.speed = speed;
        addRequirements(slideSubsystem);
    }

    
    @Override
    public void initialize() {
        System.out.println("ElevatorSlideCmd started");
    }

    @Override
    public void execute() {
        slideSubsystem.setSlideMotor(speed);
        System.out.println("Exec ElevatorSlideCmd: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        slideSubsystem.stopSlideMotors();
        System.out.println("ElevatorSlideCmd ended");
    }
}
