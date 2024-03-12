package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
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
	
	public static Command shootAtAngle(
		RobotContainer robot,
		Measure<Angle> angle,
		double shooterSpeed
	) {
		
		Command prepareToShoot = robot.arm.commands.holdAtAngle(angle)
			.alongWith(robot.shooter.commands.spinUp(shooterSpeed));
		
		Command waitUntilPreparedToShoot = new WaitCommand(1.5)
			.until(robot.arm.getController()::atSetpoint);
		
		Command shoot = robot.intake.commands.intake().withTimeout(1);
		
		return waitUntilPreparedToShoot
			.andThen(shoot)
			.deadlineWith(prepareToShoot)
			.finallyDo(robot.shooter::stop);
		
	}

	public static Command followTrajectory(Swerve swerveSubsystem) {

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
		
		return new InstantCommand(() -> {});

//		return swerveSubsystem.commands.drive(trajectory, swerveSubsystem.getFieldRelativeHeadingRotation2d());
	}
	
}
