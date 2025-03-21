package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.IntakeConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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

    public void setAlgaePosition(boolean consuming) {
        //CHANGE: for SparkMax
            if (consuming) {
            System.out.println("setting setPosition to consuming");
            intakeDownMotor.set(IntakeConstants.kOpenSpeed);
            intakeUpMotor.set(0.5);
        } else {
            System.out.println("setting intake to spitting");
            intakeDownMotor.set(IntakeConstants.kCloseSpeed);
            intakeUpMotor.set(-0.5);
        }
        //CHANGE: for Talon
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

    public void setCoralPosition(boolean consuming) {
        //CHANGE: for SparkMax
        if (consuming) {
            System.out.println("setting CoralPosition to consuming");
            intakeUpMotor.set(IntakeConstants.kOpenSpeed);
        } else {
            System.out.println("setting CoralPosition to spitting");
            intakeUpMotor.set(IntakeConstants.kCloseSpeed);
        }
        //CHANGE: for Talon
        // if (open) {
        //     System.out.println("setting intake to open");
        //     intakeUpMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
        // } else {
        //     System.out.println("setting intake to closed");
        //     intakeUpMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
        // }
    }
    
    public void setConsuming() {
        this.setAlgaePosition(true);
        this.setCoralPosition(true);
    }

    public void setSpitting() {
        this.setAlgaePosition(false);
        this.setCoralPosition(false);
    }

    public void stopMotors() {
        intakeUpMotor.set(0);
        intakeDownMotor.set(0);
    }
}
