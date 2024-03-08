package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class ComplexCommands {
	
	public static Command prepareToShootAtAngle(
		RobotContainer robot,
		Measure<Angle> angle,
		double shooterSpeed
	) {
		
		return robot.arm.commands.goToRestingAngle(angle)
			.alongWith(robot.shooter.commands.spinUp(shooterSpeed));
		
	}
	
	public static Command finishShootingAtAngle(
		RobotContainer robot,
		Measure<Angle> angle,
		double shooterSpeed
	) {
		
		return robot.intake.commands.intake().raceWith(new WaitCommand(1))
			/*.onlyIf(
				robot.arm.triggers.armHasReachedSetpoint()
//					.and(robot.shooter.triggers.isAtSpeed(shooterSpeed))
			)*/.andThen(
				robot.arm.commands.goToRestingAngle(Degrees.of(0))
					.alongWith(robot.shooter.commands.stop())
			);
		
	}
	
	public static Command shootAtAngle(
		RobotContainer robot,
		Measure<Angle> angle,
		double shooterSpeed
	) {
		
		return ComplexCommands.prepareToShootAtAngle(
			robot,
			angle,
			shooterSpeed
		).andThen(ComplexCommands.finishShootingAtAngle(
			robot,
			angle,
			shooterSpeed
		));
		
	}
	
//	public static Command crossLine() {
//
//		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//			FeetPerSecond.of(1),
//			FeetPerSecond.of(0.5).per(Units.Second)
//		);
//
//		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//			new Pose2d(0, 0, new Rotation2d(0)),
//			List.of(new Translation2d(1, 0)),
//			new Pose2d(2, 0, new Rotation2d(0)),
//			trajectoryConfig
//		);
//
//	}
	
}
