package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDropCoralCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final boolean open;

    public IntakeDropCoralCmd(IntakeSubsystem intakeSubsystem, boolean open) {
        this.open = open;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeDropCoralCmd started!");
    }

    @Override
    public void execute() {
        //System.out.println("executing IntakeSetCmd!");
        intakeSubsystem.setCoralPosition(false);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        System.out.println("IntakeDropCoralCmd ended!");
    }
}
