// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class StartPositions {
    public enum StartPosition {

        MIDDLE (0),

        AMP_SIDE (.35),

        FAR_SIDE (-.35);

        Alliance alliance = DriverStation.getAlliance().get();
        public final double autonYSpeed;
        StartPosition (double autonYSpeed) {

            if (alliance == Alliance.Red)
                this.autonYSpeed = -autonYSpeed;

            else 
                this.autonYSpeed = autonYSpeed;
                
        }
    }
}
