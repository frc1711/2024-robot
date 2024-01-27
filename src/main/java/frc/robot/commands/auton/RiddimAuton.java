// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auton;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.auton.framework.PickupAuton;
// import frc.robot.commands.auton.framework.SpeakerAuton;
// import frc.robot.commands.auton.framework.basic.ArmAuton;
// import frc.robot.commands.auton.framework.basic.OdometryAuton;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.swerve.Swerve;
// import frc.robot.util.Odometry;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class RiddimAuton extends SequentialCommandGroup {
  
//   public RiddimAuton(Odometry kinematics, Swerve swerveSubsystem, Intake intakeSubsystem, Shooter shooterSubsystem, Arm armSubsystem) {
//     //TODO: Decide which notes to pick up in autons to determine angle to the speaker's AprilTag and their positions relative to the robot
//     addCommands(
//       new SpeakerAuton(shooterSubsystem, armSubsystem, 0), 
//       new OdometryAuton(kinematics, swerveSubsystem, new Translation2d()), 
//       new PickupAuton(intakeSubsystem, swerveSubsystem), 
//       new SpeakerAuton(shooterSubsystem, armSubsystem, 0), 
//       new OdometryAuton(kinematics, swerveSubsystem, new Translation2d()),
//       new PickupAuton(intakeSubsystem, swerveSubsystem),
//       new OdometryAuton(kinematics, swerveSubsystem, new Translation2d()),
//       new ArmAuton(armSubsystem, 90));
//   }
// }
