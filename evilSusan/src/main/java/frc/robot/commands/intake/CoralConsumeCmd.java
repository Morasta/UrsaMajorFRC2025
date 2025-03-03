package frc.robot.commands.intake;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class CoralConsumeCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final boolean consuming;

    public CoralConsumeCmd(IntakeSubsystem intakeSubsystem, boolean consuming) {
        this.consuming = consuming;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CoralConsumeCmd started!");
    }

    @Override
    public void execute() {
        System.out.println("executing CoralConsumeCmd!");
        intakeSubsystem.setCoralPosition(consuming);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        System.out.println("CoralConsumeCmd ended!");
    }
}
