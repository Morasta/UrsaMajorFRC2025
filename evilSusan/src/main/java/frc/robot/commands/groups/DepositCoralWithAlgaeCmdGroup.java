package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveForwardTillDistRightCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorVerticalAutoCmd;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.intake.AlgaeConsumeCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;

public class DepositCoralWithAlgaeCmdGroup extends SequentialCommandGroup {
    public DepositCoralWithAlgaeCmdGroup(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, SlideSubsystem slideSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        //will drive forward till distance to aprilTag is right. Spins elevator up to bottom Algae. Moves slide all the way out. Will grab an algae while also spitting out coral 
        addCommands(
            new DriveForwardCmd(driveTrain, 0).withTimeout(1)
            ,new DriveForwardTillDistRightCmd(driveTrain, visionSubsystem, 0, 0.3)
            ,new ElevatorVerticalAutoCmd(elevatorSubsystem, -1).withTimeout(0.4)
            ,new ElevatorSlideCmd(slideSubsystem, 0.3).withTimeout(1)
            ,new AlgaeConsumeCmd(intakeSubsystem, true).withTimeout(1)
            ,new ElevatorVerticalCmd(elevatorSubsystem, -0.10).withTimeout(2)
        );
    }
}
