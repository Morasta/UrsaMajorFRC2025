package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeConsumeCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final boolean consuming;

    public AlgaeConsumeCmd(IntakeSubsystem intakeSubsystem, boolean consuming) {
        this.consuming = consuming;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("AlgaeConsumeCmd started!");
    }

    @Override
    public void execute() {
        System.out.println("executing AlgaeConsumeCmd!");
        intakeSubsystem.setAlgaePosition(consuming);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        System.out.println("AlgaeConsumeCmd ended!");
    }
}
