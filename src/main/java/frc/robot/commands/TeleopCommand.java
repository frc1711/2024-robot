// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopCommand extends ParallelCommandGroup {
    /**
     * Creates a new TeleopCommand.
     */
    public TeleopCommand(Swerve swerve, XboxController driveController) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new DriveCommand(
                swerve,
                driveController::getLeftY,
                driveController::getLeftX,
                driveController::getRightX,
                driveController::getRightBumper,
                driveController::getRightStickButton,
                null,
                null
            )
        );

    }

}