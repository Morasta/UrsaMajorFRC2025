package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotChassis;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class IntakeSubsystem extends SubsystemBase {
    private final SparkMaxConfig sparkInvertedConfig = new SparkMaxConfig();
    private final SparkMax intakeUpMotor = new SparkMax(IntakeConstants.kTopMotorPort, MotorType.kBrushless);
    private final SparkMax intakeDownMotor = new SparkMax(IntakeConstants.kBottomMotorPort, MotorType.kBrushless);
    // TODO: Revise these to use the newly installed motors, talons?
    // private TalonSRX intakeUpMotor = new TalonSRX(IntakeConstants.kLeftMotorPort);
    // private TalonSRX intakeDownMotor = new TalonSRX(IntakeConstants.kRightMotorPort);

        Encoder enc;

    public IntakeSubsystem() {
        // enc = new Encoder(IntakeConstants.kLeftEncoderA, IntakeConstants.kLeftEncoderB);
        // enc = new Encoder(IntakeConstants.kRightEncoderA, IntakeConstants.kRightEncoderB);
        // enc.setDistancePerPulse(Math.PI*RobotChassis.wheelDiameter/RobotChassis.SRXMagEncoderCPR);
        sparkInvertedConfig.inverted(true);
        //intakeUpMotor.configure(sparkInvertedConfig, null, null);
    
    }

    @Override
    public void periodic() {
        double dist;
        //SmartDashboard.putNumber("Encoder", dist);
    }

    // TODO: make sure this goes to fully open or fully closed, based on encoders
    private void setPosition(boolean open) {
        //TODO: for SparkMax
            if (open) {
            System.out.println("setting intake to open");
            intakeUpMotor.set(IntakeConstants.kOpenSpeed);
            intakeDownMotor.set(IntakeConstants.kOpenSpeed);
        } else {
            System.out.println("setting intake to closed");
            intakeUpMotor.set(IntakeConstants.kCloseSpeed);
            intakeDownMotor.set(IntakeConstants.kCloseSpeed);
        }
        //TODO: for Talon
        // if (open) {
        //     System.out.println("setting intake to open");
        //     intakeUpMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
        //     intakeDownMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
        // } else {
        //     System.out.println("setting intake to closed");
        //     intakeUpMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
        //     intakeDownMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
        // }
    }

    public void setCoralPosition(boolean open) {
        //TODO: for SparkMax
        if (open) {
            System.out.println("setting intake to open");
            intakeUpMotor.set(IntakeConstants.kOpenSpeed);
        } else {
            System.out.println("setting intake to closed");
            intakeUpMotor.set(IntakeConstants.kCloseSpeed);
        }
        //TODO: for Talon
        // if (open) {
        //     System.out.println("setting intake to open");
        //     intakeUpMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
        // } else {
        //     System.out.println("setting intake to closed");
        //     intakeUpMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
        // }
    }
    
    public void setOpen() {
        this.setPosition(true);
        this.setCoralPosition(true);
    }

    public void setClosed() {
        this.setPosition(false);
        this.setCoralPosition(false);
    }

    public void stopMotors() {
        intakeUpMotor.set(0);
        intakeDownMotor.set(0);
    }
}
