package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
    // TODO: Revise this to the number of motors, and their config, that the build team uses
    //private SparkMax intakeLeftMotor = new SparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushed);
    //private SparkMax intakeRightMotor = new SparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushed);

    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
    }

    // TODO: Once the motors are updated, revise this to match.
    public void setPosition(boolean open) {
     /*    if (open) {
            intakeLeftMotor.set(IntakeConstants.kOpenSpeed);
            intakeRightMotor.set(IntakeConstants.kOpenSpeed);
        } else {
            intakeLeftMotor.set(IntakeConstants.kCloseSpeed);
            intakeRightMotor.set(IntakeConstants.kCloseSpeed);
        }*/
    }
}
