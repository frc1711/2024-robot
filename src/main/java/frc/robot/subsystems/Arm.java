// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {}

  public void stop() {

  }

  public double getAngleDegrees () {
    return 0;
  }

  public void setToAngle (double angleInDegrees) {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}