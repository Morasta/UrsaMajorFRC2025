package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotChassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax m_slideMotor = new SparkMax(ElevatorConstants.kSlideMotorPort, MotorType.kBrushed);
    private final SparkMax m_verticalMotor = new SparkMax(ElevatorConstants.kVerticalMotorPort, MotorType.kBrushed);

    //private final SparkRelativeEncoder sparkEncoder = m_verticalMotor.get
    Encoder enc;

    public ElevatorSubsystem() {
        enc = new Encoder(ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);
        enc.setDistancePerPulse(Math.PI*RobotChassis.wheelDiameter/RobotChassis.SRXMagEncoderCPR);

        //sparkEncoder.getPosition();
    }

    @Override
    public void periodic() {
        double dist = enc.getDistance();
        //SmartDashboard.puNumber("Encoder", dist);
        SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    }

    public void setSlideMotor(double speed) { 
        m_slideMotor.set(speed);
    }

    public void setVerticalMotor(double speed) {
        m_verticalMotor.set(speed);
    }

    public double getEncoderMeters() {
        return enc.get() * ElevatorConstants.kEncoderTick2Meter;
    }

    public void setVerticalPosition(double targetPosition) {
        double pidOutput = pid.calculate(getPosition(), targetPosition);

        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        
        double motorOutput = pidOutput + GRAVITY_COMPENSATION;

        // Clamp the output to valid range
        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);

        motor.set(motorOutput);

    }

    
}
