package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SlideSubsystem;
import frc.robot.Constants.SlideConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;


public class ElevatorSlideRetractedCommand extends Command{
    private final SlideSubsystem slideSubsystem;
    private final double speed;

    public ElevatorSlideRetractedCommand(SlideSubsystem slideSubsystem, double speed) {
        this.slideSubsystem = slideSubsystem;
        this.speed = speed;
        addRequirements(slideSubsystem);
    }

    public ElevatorSlideRetractedCommand(SlideSubsystem slideSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.slideSubsystem = slideSubsystem;
        this.speed = speed;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorSlideRetractedCommand started");
    }

    @Override
    public void execute() {
        slideSubsystem.setSlidePosition(ElevatorConstants.kRetractedPosition);
        System.out.println("Exec ElevatorSlideRetractedCommand: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        slideSubsystem.stopSlideMotors();
        System.out.println("ElevatorSlideRetractedCommand ended");
    }
}
