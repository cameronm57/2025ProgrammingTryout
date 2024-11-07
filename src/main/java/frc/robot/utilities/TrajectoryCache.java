// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/**
 * Class that defines and caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */
public class TrajectoryCache {
    private FileLog log;
   
    private static int trajectoryCount = 1;
    public TrajectoryFacing[] cache = new TrajectoryFacing[trajectoryCount];        // array of trajectories

    public enum TrajectoryType {
        barrel(0);

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TrajectoryType(int value) { this.value = value; }
    }

    /**
     * A trajectory with initial and final facing for the robot
     */
    public static class TrajectoryFacing {
        public final Rotation2d initialRotation, finalRotation;
        public final Trajectory trajectory;

        /**
         * Creates a trajectory with initial and final robot facing (rotation).
         * <p> Note that the trajectory by itself does *not* contain robot facings.  The Pose2d angles in the
         * trajectory are the direction of the velocity vector.
         * <p> This object adds initial and final facings for the trajectory.
         * @param initialRotation Expected facing (rotation) of robot at beginning of trajectory
         * @param finalRotation Desired facing (rotation) of robot at end of trajectory
         * @param trajectory The trajectory
         */
        public TrajectoryFacing(Rotation2d initialRotation, Rotation2d finalRotation, Trajectory trajectory) {
            this.initialRotation = initialRotation;
            this.finalRotation = finalRotation;
            this.trajectory = trajectory;
        }

        /**
         * Returns the intial robot pose (position and facing) for the robot prior to running the trajectory
         * @return initial Pose2d
         */
        public Pose2d getInitialPose() {
            return new Pose2d( trajectory.getInitialPose().getTranslation(), initialRotation);
        }
    }

    /**
     * Builds a single trajectory based on the parameters passed in:
     * @param trajName name of the trajectory
     * @param maxVelRatio maximum velocity multiplier between 0 and 1
     * @param maxAccelRatio maximum acceleration multiplier between 0 and 1
     * @param startPose Pose2d starting position (coordinates and angle).  Angle is direction of the initial velocity vector, not the direction that the robot is facing.
     * @param interriorWaypoints List of Translation 2d waypoints (just coordinates)
     * @param endPose Pose2d ending position (coordinates and angle).  Angle is direction of the final velocity vector, not the direction that the robot is facing.
     * @param log log file
     * @return trajectory that is generated
     */
    public static Trajectory calcTrajectory(String trajName, double maxVelRatio, double maxAccelRatio, 
        Pose2d startPose, List<Translation2d> interriorWaypoints, Pose2d endPose, FileLog log) {
		Trajectory trajectory = null;
	
    	try {

			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"maxSpeed", SwerveConstants.kFullSpeedMetersPerSecond * maxVelRatio,
				"maxAcceleration", SwerveConstants.kFullAccelerationMetersPerSecondSquare * maxAccelRatio);

			// Create config for trajectory
            TrajectoryConfig config = new TrajectoryConfig(SwerveConstants.kFullSpeedMetersPerSecond * maxVelRatio,
				SwerveConstants.kFullAccelerationMetersPerSecondSquare * maxAccelRatio)
				.setKinematics(DriveConstants.kDriveKinematics);

            // Generate the trajectory
			trajectory = TrajectoryGenerator.generateTrajectory(
				startPose, interriorWaypoints, endPose, config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, "SUCCESS", true);
		};
	
		return trajectory;
	}

    /**
     * Builds a single trajectory based on the parameters passed in:
     * @param trajName name of the trajectory
     * @param maxVelRatio maximum velocity multiplier between 0 and 1
     * @param maxAccelRatio maximum acceleration multiplier between 0 and 1
     * @param startPose Pose2d starting position (coordinates and angle).  Angle is direction of the initial velocity vector, not the direction that the robot is facing.
     * @param interriorWaypoints List of Translation 2d waypoints (just coordinates)
     * @param endPose Pose2d ending position (coordinates and angle).  Angle is direction of the final velocity vector, not the direction that the robot is facing.
     * @return trajectory that is generated
     */
    private Trajectory calcTrajectory(String trajName, double maxVelRatio, double maxAccelRatio, 
        Pose2d startPose, List<Translation2d> interriorWaypoints, Pose2d endPose) {
        return calcTrajectory(trajName, maxVelRatio, maxAccelRatio, startPose, interriorWaypoints, endPose, log);
    }

    /**
     * Build all trajectories in this.cache[] for trajectory-following commands.
     * @param log
     */
    public TrajectoryCache(FileLog log){
        this.log = log;

        cache[TrajectoryType.barrel.value] = new TrajectoryFacing(
            new Rotation2d(0), // Start facing +X direction
            new Rotation2d(0), //End facing -X direction
            calcTrajectory("Barrel", 0.4, 0.4, 
            new Pose2d(Units.inchesToMeters(43), Units.inchesToMeters(90), new Rotation2d(0.0)),
            List.of(
                    new Translation2d(Units.inchesToMeters(130), Units.inchesToMeters(90)),
                    new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(20)),
                    new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(130), Units.inchesToMeters(105)), //should have gone around first barrel by this point
                    new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(105)),
                    new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(85)),
                    new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(160)),
                    new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(160)),
                    new Translation2d(Units.inchesToMeters(220), Units.inchesToMeters(90)), //should have gone around second barrel by this point
                    new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(20)),
                    new Translation2d(Units.inchesToMeters(340), Units.inchesToMeters(90))
                    ),
            new Pose2d(0.9, 2.286, new Rotation2d(0.0))
            )
        );
    }

}