package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class AutonomousCommands {
	
	public static Command crossLine() {
		
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
			FeetPerSecond.of(1),
			FeetPerSecond.of(0.5).per(Units.Second)
		);
		
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			new Pose2d(0, 0, new Rotation2d(0)),
			List.of(new Translation2d(1, 0)),
			new Pose2d(2, 0, new Rotation2d(0)),
			trajectoryConfig
		);
		
	}
	
}
