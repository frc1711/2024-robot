// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.DoublePreference;

import java.util.stream.Stream;

public class Shooter extends SubsystemBase {
	
	protected final CANSparkMax leftShooterMotorController;
	
	protected final CANSparkMax rightShooterMotorController;
	
	public final Shooter.Commands commands;
	
	public final Shooter.Triggers triggers;
	
	public Shooter() {
		
		this.leftShooterMotorController = new CANSparkMax(
			CANDevice.LEFT_SHOOTER_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
		this.rightShooterMotorController = new CANSparkMax(
			CANDevice.RIGHT_SHOOTER_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
		this.commands = new Shooter.Commands();
		this.triggers = new Shooter.Triggers();
		
		this.leftShooterMotorController.setInverted(true);
		this.rightShooterMotorController.setInverted(false);
		
		this.getMotorControllerStream().forEach((motorController) -> {
			
			// Set the idle mode to coast, so that the motors
			// don't resist rotation when they're not being
			// powered.
			motorController.setIdleMode(IdleMode.kCoast);
			
			// Set the smart stall current limit at 60A, and
			// the free speed smart current limit at 100A.
			motorController.setSmartCurrentLimit(60, 100);
			
		});
		
		this.getMotorControllerStream()
			.map(CANSparkMax::getPIDController)
			.forEach((pidController) -> {
				
				pidController.setP(0.05);
				pidController.setD(0);
				
			});
		
	}
	
	public Stream<CANSparkMax> getMotorControllerStream() {
		
		return Stream.of(this.leftShooterMotorController, this.rightShooterMotorController);
		
	}
	
	public void stop() {
		
		this.getMotorControllerStream().forEach(CANSparkMax::stopMotor);
		
	}
	
	public void shoot() {
		
		DoublePreference preference = DoublePreference.SHOOTER_SPEED;
		
		this.shoot(Preferences.getDouble(preference.key, preference.defaultValue));
		
	}
	
	public void shoot(double speed) {
		
		this.getMotorControllerStream().forEach(
			(motorController) -> motorController.set(speed)
		);
		
	}
	
	public class Commands {
		
		public Command shoot() {
			
			return Shooter.this.startEnd(
				Shooter.this::shoot,
				Shooter.this::stop
			);
			
		}
		
		public Command shoot(double speed) {
			
			return Shooter.this.startEnd(
				() -> Shooter.this.shoot(speed),
				Shooter.this::stop
			);
			
		}
		
		public Command spinUp() {
			
			return new FunctionalCommand(
				Shooter.this::shoot,
				() -> {},
				(Boolean wasInterrupted) -> {
					if (wasInterrupted) Shooter.this.stop();
				},
				() ->
					Shooter.this.leftShooterMotorController.getAppliedOutput() >= 1 &&
						Shooter.this.rightShooterMotorController.getAppliedOutput() >= 1,
				Shooter.this
			);
			
		}
		
		public Command spinUp(double speed) {
			
			return new FunctionalCommand(
				() -> Shooter.this.shoot(speed),
				() -> {},
				(wasInterrupted) -> {},
				() ->
					Shooter.this.leftShooterMotorController.getAppliedOutput() >= (speed * 0.95) &&
					Shooter.this.rightShooterMotorController.getAppliedOutput() >= (speed * 0.95),
				Shooter.this
			);
			
		}
		
		public Command stop() {
			
			return Shooter.this.runOnce(Shooter.this::stop);
			
		}
		
	}
	
	public class Triggers {
		
		public Trigger isAtSpeed() {
			
			return this.isAtSpeed(1);
			
		}
		
		public Trigger isAtSpeed(double speed) {
			
			return new Trigger(() ->
				Shooter.this.leftShooterMotorController.getAppliedOutput() >= (speed * 0.95) &&
				Shooter.this.rightShooterMotorController.getAppliedOutput() >= (speed * 0.95)
			);
			
		}
		
	}
	
}
