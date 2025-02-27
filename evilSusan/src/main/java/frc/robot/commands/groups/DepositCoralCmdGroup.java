package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.intake.IntakeSetOpenCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DepositCoralCmdGroup extends SequentialCommandGroup {
    public DepositCoralCmdGroup(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            new ElevatorVerticalCmd(elevatorSubsystem, 1.0)
            , new ElevatorSlideCmd(elevatorSubsystem, 1.0)
            , new IntakeSetOpenCmd(intakeSubsystem, true)
        );
    }
}
