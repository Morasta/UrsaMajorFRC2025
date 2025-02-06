package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase{
    //private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.kMotorPort, MotorType.kBrushed);
    private Encoder encoder = new Encoder(//
            ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);

    public ElevatorSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    }

    public void setMotor(double speed) {
       // elevatorMotor.set(speed);
    }

    public double getEncoderMeters() {
        return encoder.get() * ElevatorConstants.kEncoderTick2Meter;
    }
}
