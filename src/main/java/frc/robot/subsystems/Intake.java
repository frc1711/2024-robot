// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  
  CANSparkMax intakeMotor;

  public Intake(int intakeMotorID) {
    intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void stop () {
    intakeMotor.stopMotor();
  }

  double intakeMotorSpeed = .5;
  public void runIntake () {
    intakeMotor.set(intakeMotorSpeed);
  }

  public void increaseIntakeSpeed () {
    if (intakeMotorSpeed <= 1)
    intakeMotorSpeed += .05;
  }

  public void decreaseIntakeSpeed () {
    if (intakeMotorSpeed >= -1)
    intakeMotorSpeed -= .05;
  }

  public boolean isHoldingNote () {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
