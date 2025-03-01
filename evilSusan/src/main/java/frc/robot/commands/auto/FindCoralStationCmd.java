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
import frc.robot.commands.drive.DriveLeftSidewaysCmd;
import frc.robot.commands.drive.DriveRightSidewaysCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.lib.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.Constants.LimelightVisionConstants.LimelightCamera;
// import limelight.networktables.LimelightSettings.LEDMode;

public class FindCoralStationCmd extends Command {

    private final DriveTrain driveSubsystem;
    private final LimelightVisionSubsystem visionSubsystem;
    private final double distance;
    private final int targetID;

    // limelight.pipelineSwitch(0);

    private void printStatus(String stateStatus) {
        System.out.println(this.getClass().getSimpleName() + " " + stateStatus);
    }

    // public FindCoralStationCmd(DriveTrain driveTrain, LimelightVisionSubsystem
    // visionSubsystem, double distance) {
    // this(driveTrain, visionSubsystem, distance, 1.0);
    // System.out.println("FindCoralStationCmd Called");
    // }

    public FindCoralStationCmd(DriveTrain driveTrain, LimelightVisionSubsystem visionSubsystem, double distance,
            double speed, int targetID) {
        printStatus("Created");
        this.driveSubsystem = driveTrain;
        this.visionSubsystem = visionSubsystem;
        this.distance = 1; // TODO: Fix me
        this.targetID = targetID;
        // * this.distance = DriveTrain.getEncoderMeters() + distance; */
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
        // .withLimelightLEDMode(LEDMode.PipelineControl)
        // .withCameraOffset(Pose3d.kZero)
        // .save();

        // printStatus("executed" +
        // visionSubsystem.limelight.getLatestResults().toString());

        // Testing functions:
        System.out.println("Distance to target: " + getDistanceToTarget(20));
        alignToAprilTag();

        // Align or drive until min distance
        // align target, then drive forward repeatedly
        // until close to target
        double distance = getDistanceToTarget(targetID);
        while (distance > 6) { // TODO: Update to the correct number of inches that we want to be away from the
                               // target
            alignToAprilTag();
            driveSubsystem.runOnce(() -> new DriveForwardCmd(driveSubsystem, 1, 0.3));
            distance = getDistanceToTarget(targetID); // update distance for next iteration of loop
        }

        for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
            System.out.println("In for loop. fID: " + target.fiducialID);
            switch ((int) target.fiducialID) {
                case Red.ReefTopRight:
                    System.out.println("Reading AprilTag 20");
                    // driveSubsystem.runOnce(() -> new DriveForwardCmd(driveSubsystem,
                    // AprilTagDists.ToReefStation));
                    break;
                case Red.ReefRight:
                    System.out.println("Reading AprilTag 21");
                    // driveSubsystem.runOnce(() -> new DriveBackwardCmd(driveSubsystem,
                    // AprilTagDists.ToReefStation));
                    break;
                case Red.ReefBottomRight:
                    System.out.println("Reading AprilTag 22");
                    // driveSubsystem.runOnce(() -> new DriveRoundTurnCmd(driveSubsystem,
                    // AprilTagDists.ToReefStation));
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
        // return Math.abs(lastValidTargetTY - targetDistance) < distanceTolerance &&
        // Math.abs(lastValidTargetAngle - targetAngle);
        return false;
    }

    // Ty: how far up or down target is
    // Tx: how far left or right the target is
    // Ta: area / how big the target looks (0%-100% of the image)
    // AprilTag is located 6-7/8 inches off the ground

    // From
    // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
    // Replace with the "area" solution if necessary
    public double getDistanceToTarget(int targetId) {
        double targetOffsetAngle_Vertical = visionSubsystem.getYValue();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; // TODO: update

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0; // TODO: update to height of where camera is mounted

        // distance from the target to the floor
        double goalHeightInches = 6.875;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance (distanceFromLimelightToGoalInches)
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    // Robot starting position is 7ft 4in from the reef
    // public double getDistanceFromArea(int targetId) {

    // }

    // strafe robot back and forth - right sideways command (might need left
    // sideways too) and align itself until close to 0
    // drive forward, measure angle, move left right as needed to align, repeat

    // drive forwrd a second (will be part of find coral command group, don't need
    // to add here)
    // start by finding the angle, and move left or right accordingly
    // then move forward for 1 second, then do again

    /*
     * https://www.chiefdelphi.com/t/aligning-rotation-using-limelight/491164
     * https://www.chiefdelphi.com/t/limelight-autonomous-targeting-with-mecanum-
     * cartesian-drive/403206/8
     * https://www.chiefdelphi.com/t/how-to-get-mecanum-to-work-in-auton/403185/4
     */
    public void alignToAprilTag() {
        double Tx = visionSubsystem.getXValue();
        double tolerance = 0; // TODO: update tolerance

        System.out.println("alignment x value: " + Tx);
        // IF TX+tolerance > ???
        // strafe another way
        // IF TX+tolerance > ???
        // strafe one way
        if (Tx + tolerance > 1) {
            driveSubsystem.runOnce(() -> new DriveRightSidewaysCmd(driveSubsystem, 1, 0.3));
        } else if (Tx + tolerance < 1) {
            driveSubsystem.runOnce(() -> new DriveLeftSidewaysCmd(driveSubsystem, 1, 0.3));
        }
    }
}
