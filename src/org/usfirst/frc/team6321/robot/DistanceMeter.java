package org.usfirst.frc.team6321.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

class DistanceMeter {
	private ArrayList<Double> inputAccels = new ArrayList<Double>();
	private ArrayList<Double> velocities = new ArrayList<Double>();
	private ArrayList<Double> distances = new ArrayList<Double>();

	private int width = 800, height = 800;

	private double tickRate = 50.0;

	private double distance, velocity;

	private int inputs = 0;

	private BuiltInAccelerometer acc;

	private double totalVel = 0.0;
	private double averageVel = 0;

	// TIME STUFF

	private double timeChanged = 0;
	private long startTime = 0L, nowTime = 0L;

	public DistanceMeter(BuiltInAccelerometer acc) {
		this.acc = acc;
	}

	public double maxValue(ArrayList<Double> x) {
		double max = 0;
		for (double loc : x)
			if (Math.abs(loc) > max)
				max = Math.abs(loc);
		return max;
	}

	public double netAcc() {
		return Math.hypot(acc.getX(), acc.getY());
	}

	public double getNetDistance() {
		return 9.8 * distance;
	}

	public double getAverageVelocity() {
		return 9.8 * averageVel;
	}

	public void doMath() {

		if (startTime == 0L)
			startTime = System.nanoTime();

		double acc = 0.0;
		inputAccels.add((acc = netAcc()));

		inputs++;

		velocity += acc / tickRate;

		velocities.add(velocity);

		totalVel += velocity;

		averageVel = totalVel / inputs;

		distance = averageVel * (double) inputs / tickRate;

		distances.add(distance);

		timeChanged = ((nowTime = System.nanoTime()) - startTime) / 1000000000.0;

		System.out.println("At time: " + timeChanged + " Distance: " + distance + " with Velocity: " + velocity);
	}
}
