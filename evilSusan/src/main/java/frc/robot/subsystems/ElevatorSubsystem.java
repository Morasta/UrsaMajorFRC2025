package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax m_slideMotor = new SparkMax(ElevatorConstants.kSlideMotorPort, MotorType.kBrushed);
    private final SparkMax m_verticalMotor = new SparkMax(ElevatorConstants.kVerticalMotorPort, MotorType.kBrushed);


    private Encoder encoder = new Encoder(
            ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);

    public ElevatorSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    }

    public void setSlideMotor(double speed) {
        m_slideMotor.set(speed);
    }

    public void setVerticalMotor(double speed) {
        m_verticalMotor.set(speed);
    }

    public double getEncoderMeters() {
        return encoder.get() * ElevatorConstants.kEncoderTick2Meter;
    }
}
