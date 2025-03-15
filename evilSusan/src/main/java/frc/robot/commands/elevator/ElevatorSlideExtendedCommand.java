package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.Constants.SlideConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;


public class ElevatorSlideExtendedCommand extends Command{
    private final SlideSubsystem slideSubsystem;
    private final double speed;

    public ElevatorSlideExtendedCommand(SlideSubsystem slideSubsystem, double speed) {
        this.slideSubsystem = slideSubsystem;
        this.speed = speed;
        addRequirements(slideSubsystem);
    }

    public ElevatorSlideExtendedCommand(SlideSubsystem slideSubsystem, ElevatorVerticalPositions targetPosition, double speed) {
        this.slideSubsystem = slideSubsystem;
        this.speed = speed;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorSlideExtendedCommand started");
    }

    @Override
    public void execute() {
        slideSubsystem.setSlidePosition(ElevatorConstants.kExtendedPosition);
        System.out.println("Exec ElevatorSlideExtendedCommand: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        slideSubsystem.stopSlideMotors();
        System.out.println("ElevatorSlideExtendedCommand ended");
    }
}
