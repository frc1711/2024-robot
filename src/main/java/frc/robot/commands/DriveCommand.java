// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ControlsUtilities;

public class DriveCommand extends Command {

  static final double X_DEADBAND = 0.10;

  static final double Y_DEADBAND = 0.10;

  static final double THETA_DEADBAND = 0.10;

  static final double X_MAX_ACCELERATION = 0.02;

  static final double Y_MAX_ACCELERATION = 0.02;

  static final double THETA_MAX_ACCELERATION = 0.05;

  Swerve swerveSubsystem;

  double currentXSpeed;

  double currentYSpeed;

  double currentThetaSpeed;

  Timer timer;

  DoubleSupplier xSpeed, ySpeed, thetaSpeed;

  BooleanSupplier slowMode, resetEncoders, resetGyro, turnAround, xMode;

  public DriveCommand(
    Swerve swerveSubsystem,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier thetaSpeed,
    BooleanSupplier slowMode,
    BooleanSupplier resetGyro,
    BooleanSupplier turnAround,
    BooleanSupplier xMode
  ) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.thetaSpeed = thetaSpeed;
    this.slowMode = slowMode;
    this.resetGyro = resetGyro;
    this.turnAround = turnAround;
    this.xMode = xMode;
    this.currentXSpeed = 0;
    this.currentYSpeed = 0;
    this.currentThetaSpeed = 0;
    
    timer = new Timer();

    RobotContainer.putSendable("Analysis Tab", "Swerve Odometry", swerveSubsystem);

    addRequirements(swerveSubsystem);
  }

  
  @Override
  public void initialize() {
    swerveSubsystem.stop();
    timer.start();
  }

  double speedMultiplier, oneEighty, turnSpeed;
  boolean wasOneEighty;
  @Override
  public void execute() {

    double nextXSpeed = xSpeed.getAsDouble();
    double nextYSpeed = ySpeed.getAsDouble();
    double nextThetaSpeed = thetaSpeed.getAsDouble();

    // Apply individual deadbands.
    nextXSpeed = ControlsUtilities.applyDeadband(nextXSpeed, DriveCommand.X_DEADBAND);
    nextYSpeed = ControlsUtilities.applyDeadband(nextYSpeed, DriveCommand.Y_DEADBAND);
    nextThetaSpeed = ControlsUtilities.applyDeadband(nextThetaSpeed, DriveCommand.THETA_DEADBAND);

    double absoluteNextXSpeed = Math.abs(nextXSpeed);
    double absoluteNextYSpeed = Math.abs(nextYSpeed);
    double absoluteNextThetaSpeed = Math.abs(nextThetaSpeed);

    // Apply global deadband.
    if (absoluteNextXSpeed < DriveCommand.X_DEADBAND && absoluteNextYSpeed < DriveCommand.Y_DEADBAND && absoluteNextThetaSpeed < DriveCommand.THETA_DEADBAND) {
      nextXSpeed = 0;
      nextYSpeed = 0;
      nextThetaSpeed = 0;
    }
    else {
      nextXSpeed = ControlsUtilities.enforceMaximumPositiveDelta(currentXSpeed, nextXSpeed, X_MAX_ACCELERATION);
      nextYSpeed = ControlsUtilities.enforceMaximumPositiveDelta(currentYSpeed, nextYSpeed, Y_MAX_ACCELERATION);
      nextThetaSpeed = ControlsUtilities.enforceMaximumPositiveDelta(currentThetaSpeed, nextThetaSpeed, THETA_MAX_ACCELERATION);
    }

    this.speedMultiplier = slowMode.getAsBoolean() ? 0.25 : 1;

    if (turnAround.getAsBoolean() && !wasOneEighty) {

      timer.reset();
      oneEighty = 1;
      wasOneEighty = true;

    } else if (oneEighty != 0 && timer.hasElapsed(1)) {

      oneEighty = 0;
      wasOneEighty = false;

    }

    if (resetGyro.getAsBoolean()) swerveSubsystem.resetGyro();

    if (xMode.getAsBoolean()) swerveSubsystem.xMode();
    else if (
      Math.abs(nextXSpeed) > .15 ||
      Math.abs(nextYSpeed) > .15 ||
      Math.abs(nextThetaSpeed) > .15
    ) {

      this.currentXSpeed = nextXSpeed;
      this.currentYSpeed = nextYSpeed;
      this.currentThetaSpeed = nextThetaSpeed;

      swerveSubsystem.updateModules(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          this.currentXSpeed,
          this.currentYSpeed,
          this.currentThetaSpeed + oneEighty,
          swerveSubsystem.getGyroRotation()
        ),
        speedMultiplier
      );

    } else {

      this.currentXSpeed = nextXSpeed;
      this.currentYSpeed = nextYSpeed;
      this.currentThetaSpeed = nextThetaSpeed;

      swerveSubsystem.stop();
      swerveSubsystem.updateModules(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          0,
          0,
          oneEighty, 
          swerveSubsystem.getGyroRotation()
        ),
        1
      );

  }

}

  
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
