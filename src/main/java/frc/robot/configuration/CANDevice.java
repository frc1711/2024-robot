// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

public enum CANDevice {
	
	// Swerve Modules
	
	FRONT_LEFT_DRIVE_MOTOR_CONTROLLER(1),
	FRONT_LEFT_STEER_MOTOR_CONTROLLER(2),
	FRONT_LEFT_ENCODER(9),
	
	FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER(3),
	FRONT_RIGHT_STEER_MOTOR_CONTROLLER(4),
	FRONT_RIGHT_ENCODER(10),
	
	REAR_LEFT_DRIVE_MOTOR_CONTROLLER(5),
	REAR_LEFT_STEER_MOTOR_CONTROLLER(6),
	REAR_LEFT_ENCODER(11),
	
	REAR_RIGHT_DRIVE_MOTOR_CONTROLLER(7),
	REAR_RIGHT_STEER_MOTOR_CONTROLLER(8),
	REAR_RIGHT_ENCODER(12),
	
	// Shooter Motors
	
	LEFT_SHOOTER_MOTOR_CONTROLLER(14),
	RIGHT_SHOOTER_MOTOR_CONTROLLER(15),
	
	// Pivot Motors
	
	LEFT_PIVOT_MOTOR_CONTROLLER(17),
	RIGHT_PIVOT_MOTOR_CONTROLLER(16),
	
	// Intake Motors
	
	LEFT_UPPER_INTAKE_MOTOR_CONTROLLER(18),
	RIGHT_LOWER_INTAKE_MOTOR_CONTROLLER(19),
	
	SHOOTER_BEAM_BREAK_SENSOR(20);
	
	public final int id;
	
	CANDevice(int id) {
		
		this.id = id;
		
	}
	
}
