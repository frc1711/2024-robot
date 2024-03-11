package frc.robot.devicewrappers;

import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.configuration.DoublePreference;
import frc.robot.util.ControlsUtilities;

import static edu.wpi.first.units.Units.*;

public class RaptorsSparkAbsoluteEncoder implements Sendable {
	
	protected static final Unit<Angle> RAW_UNITS = Rotations;
	
	protected static final Unit<Velocity<Angle>> RAW_VELOCITY_UNITS =
		Rotations.per(Second);
	
	protected final SparkAbsoluteEncoder encoder;
	
	protected final DoublePreference zeroOffsetPreference;
	
	protected final Unit<Angle> zeroOffsetPreferenceUnits;
	
	public RaptorsSparkAbsoluteEncoder(
		SparkAbsoluteEncoder encoder,
		DoublePreference zeroOffsetPreference,
		Unit<Angle> zeroOffsetPreferenceUnits,
		boolean inverted
	) {
		
		this.encoder = encoder;
		this.zeroOffsetPreference = zeroOffsetPreference;
		this.zeroOffsetPreferenceUnits = zeroOffsetPreferenceUnits;
		
		this.encoder.setInverted(inverted);
		this.encoder.setPositionConversionFactor(1);
		this.encoder.setVelocityConversionFactor(1);
		
		this.setZeroOffsetFromPreference();
		
	}
	
	public Measure<Angle> getPosition() {
		
		return RaptorsSparkAbsoluteEncoder.RAW_UNITS.of(
			this.encoder.getPosition()
		);
		
	}
	
	public Measure<Velocity<Angle>> getVelocity() {
		
		return RaptorsSparkAbsoluteEncoder.RAW_VELOCITY_UNITS.of(
			this.encoder.getVelocity()
		);
		
	}
	
	public Measure<Angle> getZeroOffset() {
		
		return RaptorsSparkAbsoluteEncoder.RAW_UNITS.of(
			this.encoder.getZeroOffset()
		);
		
	}
	
	public void setZeroOffset(Measure<Angle> zeroOffset) {
		
		this.zeroOffsetPreference.set(
			zeroOffset.in(this.zeroOffsetPreferenceUnits)
		);
		
		this.encoder.setZeroOffset(zeroOffset.in(
			RaptorsSparkAbsoluteEncoder.RAW_UNITS
		));
		
	}
	
	public void setZeroOffsetFromPreference() {
		
		this.encoder.setZeroOffset(
			zeroOffsetPreferenceUnits.of(zeroOffsetPreference.get())
				.in(RaptorsSparkAbsoluteEncoder.RAW_UNITS)
		);
		
	}
	
	public void calibrate(Measure<Angle> currentAngle) {
		
		// Calculate the new zero offset relative to the existing one.
		Measure<Angle> newZeroOffset = RaptorsSparkAbsoluteEncoder.RAW_UNITS.of(
			this.encoder.getZeroOffset() + this.encoder.getPosition()
		);
		
		// Subtract the angle at which the arm is calibrated.
		newZeroOffset = newZeroOffset.minus(currentAngle);
		
		// Normalize the new zero offset to the range of the encoder.
		newZeroOffset = RaptorsSparkAbsoluteEncoder.RAW_UNITS.of(
			ControlsUtilities.normalizeToRange(
				newZeroOffset.in(RaptorsSparkAbsoluteEncoder.RAW_UNITS),
				0,
				1
			)
		);
		
		this.zeroOffsetPreference.set(
			newZeroOffset.in(this.zeroOffsetPreferenceUnits)
		);
		
		encoder.setZeroOffset(
			newZeroOffset.in(RaptorsSparkAbsoluteEncoder.RAW_UNITS)
		);
		
	}
	
	public void setInverted(boolean inverted) {
		
		this.encoder.setInverted(inverted);
		
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		
		builder.addDoubleProperty(
			"Position (%s)".formatted(
				RaptorsSparkAbsoluteEncoder.RAW_UNITS.name()
			),
			this.encoder::getPosition,
			null
		);
		
		builder.addDoubleProperty(
			"Position (%s)".formatted(
				this.zeroOffsetPreferenceUnits.name()
			),
			() -> this.getPosition().in(this.zeroOffsetPreferenceUnits),
			null
		);
		
		builder.addDoubleProperty(
			"Velocity (%s)".formatted(
				RaptorsSparkAbsoluteEncoder.RAW_VELOCITY_UNITS.name()
			),
			this.encoder::getVelocity,
			null
		);
		
		builder.addDoubleProperty(
			"Zero Offset (%s)".formatted(this.zeroOffsetPreferenceUnits.name()),
			() -> this.getZeroOffset().in(this.zeroOffsetPreferenceUnits),
			null
		);
		
		builder.addBooleanProperty(
			"Inverted",
			this.encoder::getInverted,
			this.encoder::setInverted
		);
		
	}
	
}
