// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.util.Odometry;

public class Swerve extends SubsystemBase {
  
  private SwerveModule 
    flModule,
    frModule,
    rlModule,
    rrModule;
  
  // private Odometry odometry;

  private SwerveDriveOdometry swerveDriveOdometry;

  private SwerveModulePosition[] modulePositions;

  private AHRS gyro;

  private SwerveDriveKinematics kinematics;

  //TODO: Find coordinates of start positions
  public enum StartPosition {
    STATION_ONE (new Translation2d (.8, 6.6), new Rotation2d (0)),
    STATION_TWO (new Translation2d (0, 0), new Rotation2d (0)),
    STATION_THREE (new Translation2d (0, 0), new Rotation2d (0));
    Translation2d translation;
    Rotation2d rotation;
    StartPosition (Translation2d translation, Rotation2d rotation) {
      this.translation = translation;
        this.rotation = rotation;
    }
  }

  public Swerve(
    SwerveModule flModule,
    SwerveModule frModule,
    SwerveModule rlModule,
    SwerveModule rrModule,
    AHRS gyro,
    StartPosition startPosition
  ) {
    this.flModule = flModule;
    this.frModule = frModule;
    this.rlModule = rlModule;
    this.rrModule = rrModule;
    this.gyro = gyro;
    modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = flModule.getPosition();
    modulePositions[1] = frModule.getPosition();
    modulePositions[2] = rlModule.getPosition();
    modulePositions[3] = rrModule.getPosition();
    // odometry = new Odometry(gyro, Odometry.StartPosition.STATION_ONE);
    kinematics = new SwerveDriveKinematics(
        flModule.motorMeters,
        frModule.motorMeters,
        rlModule.motorMeters,
        rrModule.motorMeters
    );
    swerveDriveOdometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), modulePositions, new Pose2d(startPosition.translation, getGyroRotation()));
    this.gyro = gyro;

    /**Create a new sendable field for each module*/
    RobotContainer.putSendable("Analysis Tab", "fl-Module", flModule);
    RobotContainer.putSendable("Analysis Tab", "fr-Module", frModule);
    RobotContainer.putSendable("Analysis Tab", "rl-Module", rlModule);
    RobotContainer.putSendable("Analysis Tab", "rr-Module", rrModule); 
    RobotContainer.putSendable("Analysis Tab", "gyro", gyro);
    // RobotContainer.putSendable("kinematics", odometry);

    /**Create a new sendable command to reset the encoders */
    RobotContainer.putCommand("Reset Encoders", new InstantCommand(this::resetEncoders, this), true);
    RobotContainer.putCommand("Reset Gyro", new InstantCommand(this::resetGyro, this), true);
    RobotContainer.putCommand("Reset Odometry", new InstantCommand(this::resetOdometry, this), true);
  }

  /**Runs the stop() method on each module */
  public void stop () {
    flModule.stop();
    frModule.stop();
    rlModule.stop();
    rrModule.stop();
  }

  public Pose2d updateOdometry () {
    modulePositions[0] = flModule.getPosition();
    modulePositions[1] = frModule.getPosition();
    modulePositions[2] = rlModule.getPosition();
    modulePositions[3] = rrModule.getPosition();
    return swerveDriveOdometry.update(getGyroRotation(),  new SwerveDriveWheelPositions(modulePositions));
  }

  public Pose2d getRobotPose () {
    return swerveDriveOdometry.getPoseMeters();
  }

  public void resetOdometry () {
    flModule.resetModuleDistance();
    frModule.resetModuleDistance();
    rlModule.resetModuleDistance();
    rrModule.resetModuleDistance();
  }

  public SwerveDriveWheelPositions getWheelPositions () {
    modulePositions[0] = flModule.getPosition();
    modulePositions[1] = frModule.getPosition();
    modulePositions[2] = rlModule.getPosition();
    modulePositions[3] = rrModule.getPosition();
    return new SwerveDriveWheelPositions(modulePositions);
  }

  public void resetGyro() {
    gyro.reset();
    swerveDriveOdometry.resetPosition(getGyroRotation(), modulePositions, updateOdometry());
  }

  public Rotation2d getGyroRotation () {
    return gyro.getRotation2d();
  }

  public float getGyroPitch () {
    return gyro.getPitch();
  }

  /**Runs the resetEncoder() method on each module */
  public void resetEncoders() {
    flModule.resetEncoder();
    frModule.resetEncoder();
    rlModule.resetEncoder();
    rrModule.resetEncoder();
  }

  public void updateModulesFieldRelative (ChassisSpeeds desiredVelocity, double speedMultiplier) {
    desiredVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(desiredVelocity, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredVelocity);
    flModule.update(moduleStates[0], speedMultiplier);
    frModule.update(moduleStates[1], speedMultiplier);
    rlModule.update(moduleStates[2], speedMultiplier);
    rrModule.update(moduleStates[3], speedMultiplier);
  }

  /**Updates each module using the reverse kinematics feature from SwerveDriveKinematics */
  public void updateModules (ChassisSpeeds desiredVelocity, double speedMultiplier) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredVelocity);
    flModule.update(moduleStates[0], speedMultiplier);
    frModule.update(moduleStates[1], speedMultiplier);
    rlModule.update(moduleStates[2], speedMultiplier);
    rrModule.update(moduleStates[3], speedMultiplier);
  }

  public void xMode () {
    flModule.update(new SwerveModuleState(0, new Rotation2d(135)), 1);
    frModule.update(new SwerveModuleState(0, new Rotation2d(-135)), 1);
    rlModule.update(new SwerveModuleState(0, new Rotation2d(45)), 1);
    rrModule.update(new SwerveModuleState(0, new Rotation2d(-45)), 1);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    builder.addDoubleProperty("Swerve Odometry X Component", () -> updateOdometry().getX(), null);
    builder.addDoubleProperty("Swerve Odometry Y Component", () -> updateOdometry().getY(), null);
  }
  
}