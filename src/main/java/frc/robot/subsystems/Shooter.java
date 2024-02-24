// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDevice;

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
		
		this.rightShooterMotorController.setInverted(false);
		this.leftShooterMotorController.setInverted(true);
		
		this.leftShooterMotorController.setIdleMode(IdleMode.kCoast);
		this.rightShooterMotorController.setIdleMode(IdleMode.kCoast);
		
	}
	
	public void stop() {
		
		this.leftShooterMotorController.stopMotor();
		this.rightShooterMotorController.stopMotor();
		
	}
	
	public void runShooter(double leftMotorSpeed, double rightMotorSpeed) {
		
		this.leftShooterMotorController.set(leftMotorSpeed);
		this.rightShooterMotorController.set(rightMotorSpeed);
		
	}
	
	double leftSpeed = .5, rightSpeed = .5;
	
	public void runShooterTest() {
		
		this.leftShooterMotorController.set(leftSpeed);
		this.rightShooterMotorController.set(rightSpeed);
		
	}
	
	public void increaseShooterSpeed() {
		
		if (this.leftSpeed <= 1) this.leftSpeed += .05;
		if (this.rightSpeed <= 1) this.rightSpeed += .05;
		
	}
	
	public void decreaseShooterSpeed() {
		
		if (this.leftSpeed >= -1) this.leftSpeed -= .05;
		if (this.rightSpeed >= -1) this.rightSpeed -= .05;
		
	}
	
	@Override
	public void periodic() {
		
		// This method will be called once per scheduler run
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty("Left Motor Speed", () -> this.leftSpeed, null);
		builder.addDoubleProperty("Right Motor Speed", () -> this.rightSpeed, null);
		
	}
	
}
