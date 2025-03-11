package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveForwardTillDistRightCmd;
import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.intake.AlgaeConsumeCmd;
import frc.robot.commands.intake.AlgaeSpitOutCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;

public class DepositCoralAfterAlgaeCmdGroup extends SequentialCommandGroup {
    public DepositCoralAfterAlgaeCmdGroup(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        //will drive forward till distance to aprilTag is right. Spins elevator up. Moves slide all the way out. Will grab an algae while also spitting out coral 
        addCommands(
            new DriveForwardTillDistRightCmd(driveTrain, visionSubsystem, 0, 0.4),
            new ElevatorVerticalCmd(elevatorSubsystem, -1).withTimeout(1),
            new ElevatorSlideCmd(elevatorSubsystem, 0.3).withTimeout(1),
            new AlgaeConsumeCmd(intakeSubsystem, true).withTimeout(1)
        );
    }
}
