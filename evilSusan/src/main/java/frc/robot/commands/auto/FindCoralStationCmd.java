package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.AprilTagDists;
import frc.robot.Constants.AutoConstants.TargetTagsCoralStation.Red;
import frc.robot.commands.drive.DriveBackwardCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.lib.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.Constants.LimelightVisionConstants.LimelightCamera;
// import limelight.networktables.LimelightSettings.LEDMode;

public class FindCoralStationCmd extends Command{

    private final DriveTrain driveSubsystem;
    private final LimelightVisionSubsystem visionSubsystem;
    private final double distance;

    // limelight.pipelineSwitch(0);

    private void printStatus(String stateStatus){
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    // public FindCoralStationCmd(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, double distance) {
        //this(driveTrain, visionSubsystem, distance, 1.0);
    //     System.out.println("FindCoralStationCmd Called");
    // }

    public FindCoralStationCmd(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, double distance, double speed) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.visionSubsystem = visionSubsystem;
        this.distance = 1; //TODO: Fix me
        //* this.distance = DriveTrain.getEncoderMeters() + distance; */
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        printStatus("init");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }

    @Override
    public void execute() {   
        driveSubsystem.feed(); 
        LimelightResults results = LimelightHelpers.getLatestResults(LimelightCamera.CAMERA_NAME);

        System.out.println(visionSubsystem.getXValue());
        System.out.println(visionSubsystem.getYValue());
        System.out.println(visionSubsystem.getAreaValue());
        // System.out.println("ty value: " + y);
        // System.out.println("ta value: " + area);
        // visionSubsystem.limelight.getSettings()
        //     .withLimelightLEDMode(LEDMode.PipelineControl)
        //     .withCameraOffset(Pose3d.kZero)
        //     .save();

            
        //printStatus("executed" + visionSubsystem.limelight.getLatestResults().toString());

        for (LimelightTarget_Fiducial target: results.targets_Fiducials) {
            System.out.println("In for loop. fID: " + target.fiducialID);
            switch ((int)target.fiducialID) {
                case Red.ReefTopRight:
                    System.out.println("Reading AprilTag 20");
                    //driveSubsystem.runOnce(() -> new DriveForwardCmd(driveSubsystem, AprilTagDists.ToReefStation));
                    break;
                case Red.ReefRight:
                    System.out.println("Reading AprilTag 21");
                    //driveSubsystem.runOnce(() -> new DriveBackwardCmd(driveSubsystem, AprilTagDists.ToReefStation));
                    break;
                case Red.ReefBottomRight:
                    System.out.println("Reading AprilTag 22");
                    //driveSubsystem.runOnce(() -> new DriveRoundTurnCmd(driveSubsystem, AprilTagDists.ToReefStation));
                    break;
                default:
                    System.out.println("Default case");
                    break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("end");
        System.out.println(this.getClass().getSimpleName() + " executed");
    }

    @Override
    public boolean isFinished() {
        //return Math.abs(lastValidTargetTY - targetDistance) < distanceTolerance && Math.abs(lastValidTargetAngle - targetAngle);
        return false;
    }

    // From https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
    // Replace with the "area" solution if necessary
    public double getDistanceToTarget(int targetId) {
        double targetOffsetAngle_Vertical = visionSubsystem.getYValue();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 
    
        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0; 
    
        // distance from the target to the floor
        double goalHeightInches = 60.0; 
    
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    
        //calculate distance (distanceFromLimelightToGoalInches)
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    /*  
        https://www.chiefdelphi.com/t/aligning-rotation-using-limelight/491164
        https://www.chiefdelphi.com/t/limelight-autonomous-targeting-with-mecanum-cartesian-drive/403206/8
        https://www.chiefdelphi.com/t/how-to-get-mecanum-to-work-in-auton/403185/4 
    */
    public void alignToAprilTag() {
        // IF TX+tolerance > ???
            //strafe another way
        // IF TX+tolerance > ???
            //strafe one way
    }
}
