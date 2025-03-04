package frc.robot.commands.intake;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class AlgaeSpitOutCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final boolean consuming;

    public AlgaeSpitOutCmd(IntakeSubsystem intakeSubsystem, boolean consuming) {
        this.consuming = consuming;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("AlgaeSpitOutCmd started");
    }

    @Override
    public void execute() {
        System.out.println("exec AlgaeSpitOutCmd");
        intakeSubsystem.setAlgaePosition(false);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        System.out.println("AlgaeSpitOutCmd ended");
    }
}
