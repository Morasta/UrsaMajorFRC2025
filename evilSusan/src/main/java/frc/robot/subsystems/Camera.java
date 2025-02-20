package frc.robot.subsystems;

//limelight 3A example
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class Camera {

    Thread m_visionThread;

    private final MecanumDrivePoseEstimator m_poseEstimater = new MecanumDrivePoseEstimator(
    null, m_gyro.getRotation2d(), getCurrentWheelPositions(), getPose());

    public void camera () {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
    }
}
