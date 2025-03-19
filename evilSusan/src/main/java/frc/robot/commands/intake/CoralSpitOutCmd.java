package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralSpitOutCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final boolean consuming;

    public CoralSpitOutCmd(IntakeSubsystem intakeSubsystem, boolean consuming) {
        this.consuming = consuming;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CoralSpitOutCmd started");
    }

    @Override
    public void execute() {
        System.out.println("exec CoralSpitOutCmd");
        intakeSubsystem.setCoralPosition(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        System.out.println("CoralSpitOutCmd ended");
    }
}
