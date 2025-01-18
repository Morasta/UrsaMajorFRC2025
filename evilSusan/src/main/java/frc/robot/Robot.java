// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//package edu.wpi.first.wpilibj.examples.gettingstarted;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.GameControllers.ControllerTypes;
import frc.robot.GameControllers.GameController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final PWMSparkMax m_leftDrive = new PWMSparkMax (0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
      //private final XboxController m_controller = new XboxController(0);
  private final GameController m_controller = new GameController(ControllerTypes.LogiF310);    
  private final Timer m_timer = new Timer();

  //private final Drivetrain m_drive = new Drivetrain();
  //private final XboxController driverController_HID = m_controller.getHID();

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private double aStartTime;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Possibly need to change from rightDrive
    // to LeftDrive in the future depending on orientation.
    m_rightDrive.setInverted(true);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_leftDrive.set(0);
    m_rightDrive.set(0);
    m_robotDrive.feed();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_robotDrive.feed();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    aStartTime = Timer.getFPGATimestamp();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Diffrent Message " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { 
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - aStartTime < 3) {
      //System.out.println("Moving Forward");
      m_leftDrive.set(0.5);
      m_rightDrive.set(0.5);
    } else {
      //System.out.println("Stopping");
      m_leftDrive.set(0);
      m_rightDrive.set(0);
    }
    switch (m_autoSelected) {
      case kCustomAuto:
       
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_robotDrive.arcadeDrive(0, 0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_leftDrive.set(0);
    //m_rightDrive.set(0);
    /* Axis [0] Left joystick, x-axis left (-1) and right (1)
     * Axis [1] Left joystick, y-axis up (-1) and down (1)
     * Axis [2] Left rear trigger (1-pressed)
     * Axis [3] Right rear trigger (1-pressed)
     * Axis [4] Right joystick, x-axis left (-1) and right (1)
     * Axis [5] Right joystick, y-axis up (-1) and down (1)
     * 
    */
    final double xSpeed = -m_speedLimiter.calculate(m_controller.xc.getLeftY()); //* Drivetrain.kMaxSpeed;
    final double rot = m_rotLimiter.calculate(m_controller.xc.getRightX()); //+ Drivetrain.kAngularSpeed;
    //m_robotDrive.arcadeDrive(xSpeed, 0);
    //System.out.println("In teleopPeriodic: " + xSpeed);
    //System.out.println("In teleopPeriodic: " + m_controller.getAxisCount());
    m_robotDrive.arcadeDrive(xSpeed, rot);
    //m_controller.xc.button(0).whileTrue(new InstantCommand(() -> System.out.println("Button 1 pressed")));
    m_controller.xc.getHID().getXButtonPressed();
    System.out.println("x pressed: " + m_controller.xc.getHID().getXButton());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
