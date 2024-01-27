// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.framework;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.framework.basic.ArmAuton;
import frc.robot.commands.auton.framework.basic.ShooterAuton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerAuton extends SequentialCommandGroup {
  
  double cameraToTagX, armToTagX, cameraToTagZ, armToTagZ;

  double cameraToArmX = 0;
  double cameraToArmZ = 0;

  public SpeakerAuton(Shooter shooterSubsystem, Arm armSubsystem, double angleToSpeakerTag) {
    
    armToTagZ = Limelight.LIMELIGHT.getAprilTag().get().verticalOffset() + cameraToArmZ;
    armToTagX = Limelight.LIMELIGHT.getAprilTag().get().horizontalOffset() + cameraToArmX;

    Math.atan(armToTagZ/armToTagX);

    addCommands(new ArmAuton(armSubsystem, Math.atan(armToTagZ/armToTagX)), new ShooterAuton(shooterSubsystem));
    //TODO: Run extensive troubleshooting and testing bc this is very experimental/theoretical 
  }
}
