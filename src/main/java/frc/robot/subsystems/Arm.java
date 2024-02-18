// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  
  CANSparkMax leftTuffbox, rightTuffbox;

  PIDController anglePID;

  SparkAbsoluteEncoder boreEncoder;

  public Arm(int leftTuffboxID, int rightTuffboxID) {
    leftTuffbox = new CANSparkMax(leftTuffboxID, MotorType.kBrushless);
    rightTuffbox = new CANSparkMax(rightTuffboxID, MotorType.kBrushless); 
    boreEncoder = rightTuffbox.getAbsoluteEncoder(Type.kDutyCycle);
    leftTuffbox.setIdleMode(IdleMode.kBrake);
    rightTuffbox.setIdleMode(IdleMode.kBrake);
    rightTuffbox.setInverted(true); //Set the motor to be inverted so that it rotates in the same direction as the otehr motor with the same input value.
    anglePID = new PIDController(.01, 0, 0);
  }

  public void stop() {
    leftTuffbox.stopMotor();
    rightTuffbox.stopMotor();
  }

  public double getAngleDegrees () {
    return boreEncoder.getPosition() - boreEncoder.getZeroOffset();
  }

  double rotationSpeed, motorSpeeds;
  public void setToAngle (double angleInDegrees) {
    rotationSpeed = anglePID.calculate(getAngleDegrees(), angleInDegrees);
    motorSpeeds = rotationSpeed / 2;

    leftTuffbox.set(motorSpeeds);
    rightTuffbox.set(motorSpeeds);
  }

  public void changeAngle (double speed) {
    if (getAngleDegrees() <= 90 && speed >= 0) {
    leftTuffbox.set(speed);
    rightTuffbox.set(speed);
    }
    else if (getAngleDegrees() >= 0 && speed <= 0) {
      leftTuffbox.set(speed);
      rightTuffbox.set(speed);
    }
    else stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
