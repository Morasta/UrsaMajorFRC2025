package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.CommandGroup;

import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.intake.IntakeSetOpenCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DepositCoralCmdGroup extends CommandGroup {
    public DepositCoralCmdGroup(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        addSequential (new ElevatorVerticalCmd(elevatorSubsystem, 1.0));
        addSequential (new ElevatorSlideCmd(elevatorSubsystem, 1.0));
        addSequential (new IntakeSetOpenCmd(intakeSubsystem, true));
    }
}
