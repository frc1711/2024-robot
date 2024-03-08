// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ComplexCommands;
import frc.robot.commands.auton.framework.basic.SwerveAuton;
import frc.robot.commands.auton.framework.basic.timed.TimeBasedSwerveAuton;
import frc.robot.constants.DoublePreference;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.StartPositions.StartPosition;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Fellowship extends SequentialCommandGroup {
  
  public Fellowship(Swerve swerve, Shooter shooter, Intake intake, Arm arm, StartPosition startPosition) {
    //TODO: Add shuffleboard preference for a delay between first shot and rollout
    addCommands(
        new WaitCommand(DoublePreference.AUTON_START_DELAY.get()),
        ComplexCommands.prepareToShootAtAngle(arm, shooter, Degrees.of(55), 1),
        ComplexCommands.finishShootingAtAngle(arm, shooter, intake, Degrees.of(55), 1).raceWith(new WaitCommand(2)),
        new WaitCommand(DoublePreference.AUTON_ROLLOUT_DELAY.get()),
        new SwerveAuton(swerve, .35, startPosition.autonYSpeed, swerve.getFieldRelativeHeadingRotation2d()).raceWith(new WaitCommand(1.15))
    );
    
  }
}
