package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveForwardTillDistRightCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorVerticalAutoCmd;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.intake.AlgaeConsumeCmd;
import frc.robot.commands.intake.CoralSpitOutCmd;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SlideConstants;
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
            , new DriveForwardTillDistRightCmd(driveTrain, visionSubsystem, 0, 0.3)
            , new ElevatorVerticalAutoCmd(elevatorSubsystem, ElevatorConstants.upSpeed).withTimeout(0.72)
            , new ElevatorSlideCmd(slideSubsystem, SlideConstants.slideOutSpeed).withTimeout(2)
            , new AlgaeConsumeCmd(intakeSubsystem, true).withTimeout(1)
            , new ElevatorSlideCmd(slideSubsystem, SlideConstants.slideInSpeed).withTimeout(2)
            , new ElevatorVerticalCmd(elevatorSubsystem, ElevatorConstants.autoSpeed).withTimeout(2)
            
        );
    }
}
