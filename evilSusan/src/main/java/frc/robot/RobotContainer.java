package frc.robot;



//import frc.robot.commands.DriveForwardCmd;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.OIConstants;
//import frc.robot.commands.ArcadeDriveCmd;
//import frc.robot.commands.ElevatorJoystickCmd;
//import frc.robot.commands.ElevatorPIDCmd;
//import frc.robot.commands.IntakeSetCmd;
//import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
//import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RobotContainer {
    //private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    //private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    //private final Joystick joystick1 = new Joystick(OIConstants.kDriverJoystickPort);

    public RobotContainer() {
        configureButtonBindings();

        /*driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, //
                () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
                () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis))//
        );*/
        //elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, 0));
        //intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));
    }

    private void configureButtonBindings() {
        //new JoystickButton(joystick1, OIConstants.kElevatorPIDRaiseButtonIdx)
        //        .whileActiveOnce(new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kRaisedPosition));
    }

    public Command getAutonomousCommand() {
        System.out.println("getting auto command");
        /*return new SequentialCommandGroup(
                new DriveForwardCmd(driveSubsystem, DriveConstants.kAutoDriveForwardDistance),
                new ParallelCommandGroup(
                        new IntakeSetCmd(intakeSubsystem, false),
                        new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kRaisedPosition)
                )
        );*/
        return new PrintCommand("Got autotonomous command!");
    }
}
