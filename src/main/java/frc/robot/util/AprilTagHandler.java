// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class AprilTagHandler {

    Limelight limelight;

    public AprilTagHandler () {
        limelight = Limelight.LIMELIGHT;
        limelight.setPipeline(0); //Set Limelight to AprilTag mode
    }

    /**Makes a two-dimensional vector displaying the displacement of the AprilTag
     * 
     * @return Vector from Limelight to AprilTag
     */
    public static Vector<N3> getVecToTag () {
        return VecBuilder.fill(
            Limelight.LIMELIGHT.getAprilTag().get().horizontalOffset(), 
            Limelight.LIMELIGHT.getAprilTag().get().verticalOffset(), 
            Math.atan(
                Limelight.LIMELIGHT.getAprilTag().get().verticalOffset() 
                / 
                Limelight.LIMELIGHT.getAprilTag().get().horizontalOffset()));
    }

    public static boolean getIsDesiredTag (int desiredTagID) {
        return Limelight.LIMELIGHT.getAprilTag().get().targetID() == desiredTagID;
    }

    
}
