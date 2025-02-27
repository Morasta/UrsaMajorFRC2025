package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.intake.IntakeSetOpenCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DepositCoralCommandGroup extends CommandGroup {

    public DepositCoralCmd(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        addSequential (new ElevatorVerticalCmd(elevatorSubsystem, double 1.0));
        addSequential (new ElevatorSlideCmd(elevatorSubsystem, double 1.0));
        addSequential (new IntakeSetOpenCmd(intakeSubsystem, true));
    }

}
