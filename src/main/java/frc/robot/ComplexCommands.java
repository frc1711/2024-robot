package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class ComplexCommands {
	
	public static Command prepareToShootAtAngle(
		Arm arm,
		Shooter shooter,
		Measure<Angle> angle,
		double shooterSpeed
	) {
		
		return arm.commands.goToRestingAngle(angle)
			.alongWith(shooter.commands.spinUp(shooterSpeed));
		
	}
	
	public static Command finishShootingAtAngle(
		Arm arm,
		Shooter shooter,
		Intake intake,
		Measure<Angle> angle,
		double shooterSpeed
	) {
		
		return intake.commands.intake().raceWith(new WaitCommand(1))
			.onlyIf(
				arm.triggers.armHasReachedSetpoint()
					.and(shooter.triggers.isAtSpeed(shooterSpeed))
			).andThen(
				arm.commands.goToRestingAngle(Degrees.of(0))
					.alongWith(shooter.commands.stop())
			);
		
	}
	
}