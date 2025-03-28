package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveForwardTillDistRightCmd;
import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorVerticalAutoCmd;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.intake.AlgaeConsumeCmd;
import frc.robot.commands.intake.AlgaeSpitOutCmd;
import frc.robot.commands.intake.CoralSpitOutCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;


public class DropCoralThenGrabAlgaeCmdGroup extends SequentialCommandGroup {
    public DropCoralThenGrabAlgaeCmdGroup(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, SlideSubsystem slideSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    //will drive forward till distance to aprilTag is right. Spins elevator up. Moves slide all the way out. Will grab an algae while also spitting out coral 
    //if lined up just right coral will be put on branch as elevator lowers
        addCommands(
            new DriveForwardTillDistRightCmd(driveTrain, visionSubsystem, 0, 0.3),
            new CoralSpitOutCmd(intakeSubsystem, true).withTimeout(1),
            new ElevatorVerticalAutoCmd(elevatorSubsystem, -1).withTimeout(0.8),
            new ElevatorSlideCmd(slideSubsystem, 0.3).withTimeout(1),
            new AlgaeConsumeCmd(intakeSubsystem, true).withTimeout(1)
            );
    }
}