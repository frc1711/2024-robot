package frc.robot.commands.auton.framework.basic;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class OdometryAuton extends Command {
	
	// Odometry kinematics;
	
	Swerve swerveDrive;
	
	Pose2d targetPose;
	
	PIDController xDistancePID, yDistancePID;
	
	double speedMultiplier;
	
	public OdometryAuton(Swerve swerveDrive, Pose2d targetPose, double speedMultiplier) {
		
		this.swerveDrive = swerveDrive;
		this.targetPose = targetPose;
		this.speedMultiplier = speedMultiplier;
		xDistancePID = new PIDController(1, 0, 0);
		yDistancePID = new PIDController(.01, 0, 0);
		yDistancePID.enableContinuousInput(0, 360);
		yDistancePID.setTolerance(1);
		xDistancePID.setTolerance(.1);
		
		addRequirements(swerveDrive);
		
	}
	
	@Override
	public void initialize() {
		
		swerveDrive.stop();
		
	}
	
	double displacementX, displacementY, speedX, speedY, speedTheta;
	
	Pose2d robotPose;
	
	@Override
	public void execute() {
		
		robotPose = swerveDrive.updateOdometry();
		
		displacementX = robotPose.getX();
		displacementY = robotPose.getY();
		
		speedX = xDistancePID.calculate(displacementX, targetPose.getX());
		speedY = xDistancePID.calculate(displacementY, targetPose.getY());
		speedTheta = yDistancePID.calculate(swerveDrive.getGyroRotation().getDegrees(), targetPose.getRotation().getDegrees());
		
		swerveDrive.updateModules(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedTheta, swerveDrive.getGyroRotation()), speedMultiplier);
		
	}
	
	@Override
	public void end(boolean interrupted) {
		
		swerveDrive.stop();
		
	}
	
	@Override
	public boolean isFinished() {
		
		return false;
		
	}
	
}
