package frc.robot.devicewrappers;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import frc.robot.constants.DoublePreference;

/**
 * An interface for a device that can measure angular position and velocity.
 *
 * All outputs should be mapped to the NWU ('northwest-up') coordinate system in
 * which counterclockwise is the positive rotational direction.
 */
public abstract class RaptorsEncoder {
	
	protected final DoublePreference zeroOffsetPreference;
	
	protected final Unit<Angle> zeroOffsetPreferenceUnits;
	
	protected RaptorsEncoder(
		DoublePreference zeroOffsetPreference,
		Unit<Angle> zeroOffsetPreferenceUnits
	) {
		
		this.zeroOffsetPreference = zeroOffsetPreference;
		this.zeroOffsetPreferenceUnits = zeroOffsetPreferenceUnits;
		
		this.setZeroOffset(this.zeroOffsetPreferenceUnits.of(
			this.zeroOffsetPreference.get()
		));
		
	}
	
	/**
	 * Returns the current angular position of this encoder, adjusted for the
	 * configured zero offset.
	 *
	 * @return The current angular position of this encoder.
	 */
	public abstract Measure<Angle> getPosition();
	
	/**
	 * Returns the current angular position of this encoder, without adjusting
	 * for the configured zero offset.
	 *
	 * @return The current angular position of this encoder, without adjusting
	 * for the configured zero offset.
	 */
	public abstract Measure<Angle> getRawPosition();
	
	/**
	 * Returns the current angular velocity of this encoder.
	 *
	 * @return The current angular velocity of this encoder.
	 */
	public abstract Measure<Velocity<Angle>> getVelocity();
	
	/**
	 * Returns the zero offset of this encoder.
	 *
	 * @return The zero offset of this encoder.
	 */
	public abstract Measure<Angle> getZeroOffset();
	
	/**
	 * Sets the zero offset of this encoder.
	 *
	 * @param zeroOffset The new zero offset of this encoder.
	 */
	public abstract void setZeroOffset(Measure<Angle> zeroOffset);
	
	/**
	 * Sets the zero offset of this encoder such that the current angular
	 * position of the encoder is equal to the given angle.
	 */
	public abstract void calibrateZeroOffset(Measure<Angle> currentAngle);
	
}
