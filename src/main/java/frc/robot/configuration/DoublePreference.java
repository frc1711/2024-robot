package frc.robot.configuration;

import edu.wpi.first.wpilibj.Preferences;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * A collection of floating point preferences that can be set and retrieved
 * from the RoboRIO across power cycles.
 */
public enum DoublePreference {
    
    FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Front Left Swerve Module Encoder Offset", -72.773),
    
    FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Front Right Swerve Module Encoder Offset", -9.756),
    
    REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Rear Left Swerve Module Encoder Offset", -71.279),
    
    REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Rear Right Swerve Module Encoder Offset", 110.918),
    
    SWERVE_STEER_PID_KP("Swerve Steer PID - Proportional Term", 0.04),
    
    SWERVE_STEER_PID_KD("Swerve Steer PID - Derivative Term", 0),
    
    SWERVE_STEER_PID_TOLERANCE_DEGREES("Swerve Steer PID - Tolerance (Degrees)", 2),
    
    SWERVE_DRIVE_PID_KP("Swerve Drive PID - Proportional Term", 0.1),
    
    SWERVE_DRIVE_PID_KD("Swerve Drive PID - Derivative Term", 0),
    
    SWERVE_HEADING_PID_KP("Swerve Heading PID - Proportional Term", 0.025),
    
    SWERVE_HEADING_PID_KD("Swerve Heading PID - Derivative Term", 0.002),
    
    SWERVE_HEADING_PID_TOLERANCE_DEGREES("Swerve Heading PID - Tolerance (Degrees)", 0),
    
    SHOOTER_SPEED("Shooter Speed", 1),
    
    INTAKE_SPEED("Intake Speed", 1),
    
    ARM_SPEED("Arm Speed", 0.75),
    
    /**
     * The offset (in degrees) that the left arm encoder should be zeroed to.
     */
    ARM_LEFT_ENCODER_ZERO_OFFSET("Arm Left Encoder Zero Offset", 54.316),
    
    /**
     * The offset (in degrees) that the right arm encoder should be zeroed to.
     */
    ARM_RIGHT_ENCODER_ZERO_OFFSET("Arm Right Encoder Zero Offset", 0.250),
    
    /**
     * The minimum angle (in degrees) that the arm should be allowed to reach.
     */
    ARM_MIN_ANGLE_DEGREES("Arm Minimum Angle (Degrees)", 15),
    
    /**
     * The maximum angle (in degrees) that the arm should be allowed to reach.
     */
    ARM_MAX_ANGLE_DEGREES("Arm Maximum Angle (Degrees)", 110),
    
    /**
     * The angle (in degrees) off of horizontal that the arm should be
     * positioned at when the robot is calibrated.
     */
    ARM_CALIBRATION_ANGLE_DEGREES("Arm Calibration Angle (Degrees)", 90),
    
    /**
     * The proportional term of the PID controller for the arm subsystem.
     */
    ARM_PID_KP("Arm PID - Proportional Term", 0.2),
    
    /**
     * The derivative term of the PID controller for the arm subsystem.
     */
    ARM_PID_KD("Arm PID - Derivative Term", 0),
    
    ARM_PID_TOLERANCE_DEGREES("Arm PID - Tolerance (Degrees)", 0.5),

    AUTON_START_DELAY("Auton Delay - Start", 0.25),

    AUTON_ROLLOUT_DELAY("Auton Delay - Roll Out", 0);
    
    /**
     * The key of the preference.
     */
    private final String key;
    
    /**
     * The default value of the preference to use if it is not already present
     * on the RoboRIO.
     */
    private final double defaultValue;
    
    /**
     * A list of listeners to notify when the value of this preference changes.
     */
    private final List<DoubleConsumer> listeners;
    
    /**
     * Initializes a new double preference with the given key and default value.
     *
     * @param key The key of the preference.
     * @param defaultValue The default value of the preference to use if it is
     * not already present on the RoboRIO.
     */
    DoublePreference(String key, double defaultValue) {
        
        this.key = key;
        this.defaultValue = defaultValue;
        this.listeners = new ArrayList<>();
        
    }
    
    /**
     * Initializes all double preferences.
     */
    public static void init() {
        
        for (DoublePreference preference: DoublePreference.values()) {
            
            Preferences.initDouble(preference.key, preference.defaultValue);
            
        }
        
    }
    
    /**
     * Returns the value of the preference.
     *
     * @return The value of the preference.
     */
    public double get() {
        
        return Preferences.getDouble(this.key, this.defaultValue);
        
    }
    
    /**
     * Sets the value of the preference.
     *
     * @param value The value to set the preference to.
     */
    public void set(double value) {
        
        Preferences.setDouble(this.key, value);
        this.listeners.forEach((listener) -> listener.accept(value));
        
    }
    
    /**
     * Registers the given listener function to handle changes to the value of
     * this preference.
     *
     * @param listener The listener function to register.
     */
    public void onChange(DoubleConsumer listener) {
        
        this.listeners.add(listener);
        
    }
    
    /**
     * Passes the value of this preference to the given consumer and registers
     * the consumer to be called whenever the value of this preference changes.
     *
     * This is a convenience method for setting the value of the preference and
     * registering a listener in one line.
     *
     * @param consumer The consumer to pass the value of this preference to.
     */
    public void useValue(DoubleConsumer consumer) {
        
        consumer.accept(this.get());
        this.onChange(consumer);
        
    }
    
}
