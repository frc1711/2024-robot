// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public enum IDMap {
	
	// Swerve Modules
	
	FRONT_LEFT_DRIVE_MOTOR(1),
	FRONT_LEFT_STEER_MOTOR(2),
	FRONT_LEFT_ENCODER(9),
	
	FRONT_RIGHT_DRIVE_MOTOR(3),
	FRONT_RIGHT_STEER_MOTOR(4),
	FRONT_RIGHT_ENCODER(10),
	
	REAR_LEFT_DRIVE_MOTOR(5),
	REAR_LEFT_STEER_MOTOR(6),
	REAR_LEFT_ENCODER(11),
	
	REAR_RIGHT_DRIVE_MOTOR(7),
	REAR_RIGHT_STEER_MOTOR(8),
	REAR_RIGHT_ENCODER(12),
	
	// Shooter Motors
	
	LEFT_SHOOTER_MOTOR(15),
	RIGHT_SHOOTER_MOTOR(14),
	
	// Pivot Motors
	
	LEFT_PIVOT_MOTOR(17),
	RIGHT_PIVOT_MOTOR(16),
	
	// Intake Motors
	
	LEFT_LOWER_INTAKE_MOTOR(18),
	RIGHT_UPPER_INTAKE_MOTOR(19);
	
	public final int id;
	
	IDMap(int id) {
		
		this.id = id;
		
	}
	
}
