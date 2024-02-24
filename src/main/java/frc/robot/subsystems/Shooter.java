// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDevice;
import frc.robot.constants.DoublePreference;

import java.util.stream.Stream;

public class Shooter extends SubsystemBase {
	
	protected final CANSparkMax leftShooterMotorController;
	
	protected final CANSparkMax rightShooterMotorController;
	
	public Shooter() {
		
		this.leftShooterMotorController = new CANSparkMax(
			CANDevice.LEFT_SHOOTER_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
		this.rightShooterMotorController = new CANSparkMax(
			CANDevice.RIGHT_SHOOTER_MOTOR_CONTROLLER.id,
			MotorType.kBrushless
		);
		
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
			
			// Set the maximum rotational acceleration to ramp at a speed
			// that would reach 100% speed from 0% speed in n seconds.
			motorController.setOpenLoopRampRate(1);
			
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
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty("Left Motor Speed", leftShooterMotorController::get, null);
		builder.addDoubleProperty("Right Motor Speed", rightShooterMotorController::get, null);
		
	}
	
	public class Commands {
		
		public Command shoot() {
			
			return Shooter.this.run(Shooter.this::shoot);
			
		}
		
	}
	
}
