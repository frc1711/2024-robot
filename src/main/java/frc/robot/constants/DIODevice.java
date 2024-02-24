package frc.robot.constants;

/**
 * Enumerates the devices plugged into the DIO ports of the RoboRIO, alongside
 * which port ID that they are plugged in to.
 */
public enum DIODevice {
    
    /**
     * The upper limit switch on the left side of the arm.
     *
     * This limit switch is normally closed.
     */
    LEFT_UPPER_ARM_LIMIT_SWITCH(9),
    
    /**
     * The upper limit switch on the right side of the arm.
     *
     * This limit switch is normally closed.
     */
    RIGHT_UPPER_ARM_LIMIT_SWITCH(8),
    
    /**
     * The lower limit switch on the left side of the arm.
     *
     * This limit switch is normally closed.
     */
    LEFT_LOWER_ARM_LIMIT_SWITCH(7),
    
    /**
     * The lower limit switch on the right side of the arm.
     *
     * This limit switch is normally closed.
     */
    RIGHT_LOWER_ARM_LIMIT_SWITCH(6);
    
    /**
     * The ID of the port that this DIODevice is plugged in to.
     */
    public final int id;
    
    /**
     * Initializes a new DIODevice instance with the given port ID.
     *
     * @param id The ID of the DIO port on the RoboRIO that this device is
     * plugged in to.
     */
    DIODevice(int id) {

        this.id = id;

    }

}
