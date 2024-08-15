package frc.robot.devicewrappers;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Units;

public class RaptorsLaserCanBooleanSupplier implements BooleanSupplier, Supplier<Boolean> {
	
	protected LaserCan laserCan;
	
	protected Function<Measure<Distance>, Boolean> testFunction;
	
	public RaptorsLaserCanBooleanSupplier(
		int deviceId,
		Function<Measure<Distance>, Boolean> testFunction
	) {
		
		this.laserCan = new LaserCan(deviceId);
		this.testFunction = testFunction;
		
		try {
			
			this.laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
			this.laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
			this.laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(
				8, 8, 6, 6
			));
			
		} catch (ConfigurationFailedException exception) {
			
			System.err.println(exception.toString());
			
		}
		
	}
	
	@Override
	public boolean getAsBoolean() {
		
		LaserCan.Measurement measurement = this.laserCan.getMeasurement();
		
		if (measurement == null) return false;
		
		return this.testFunction.apply(Units.Millimeters.of(
			this.laserCan.getMeasurement().distance_mm
		));
		
	}
	
	@Override
	public Boolean get() {
		
		return this.getAsBoolean();
		
	}
}
