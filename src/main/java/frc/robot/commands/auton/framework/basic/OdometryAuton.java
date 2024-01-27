package frc.robot.commands.auton.framework.basic;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auton.framework.basic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Odometry;

public class OdometryAuton extends Command {

  Odometry kinematics;
  Swerve swerveDrive;
  Translation2d targetPosition;
  PIDController xDistancePID, yDistancePID;
  
  public OdometryAuton(Swerve swerveDrive, Translation2d targetPosition) {
    this.swerveDrive = swerveDrive;
    this.targetPosition = targetPosition;
    xDistancePID = new PIDController(0.01, 0, 0);
    yDistancePID = new PIDController(0.01, 0, 0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    swerveDrive.stop();
  }

  double displacementX, displacementY, speedX, speedY;
  Pose2d robotPose;
  @Override
  public void execute() {
    // robotPose = swerveDrive.updateOdometry();

    displacementX = robotPose.getX();
    displacementY = robotPose.getY();

    speedX = xDistancePID.calculate(displacementX, targetPosition.getX());
    speedY = yDistancePID.calculate(displacementY, targetPosition.getY());

    swerveDrive.updateModules(new ChassisSpeeds(speedX, speedY, 0), 0);
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
