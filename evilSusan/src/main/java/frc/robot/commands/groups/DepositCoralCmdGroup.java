package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveForwardTillDistRightCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.intake.CoralSpitOutCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.DriveTrain;

public class DepositCoralCmdGroup extends SequentialCommandGroup {
    public DepositCoralCmdGroup(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        //DriveForward for half second. with read tag and drive till tag is correct distance. will CoralSpitOut
        addCommands(
            new DriveForwardCmd(driveTrain, 0).withTimeout(0.5)
            , new DriveForwardTillDistRightCmd(driveTrain, visionSubsystem, 0, 0)
            , new CoralSpitOutCmd(intakeSubsystem, false).withTimeout(1)
        );
    }
}
