package org.usfirst.frc.team6321.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {

	private double ki, kp, kd;

	private int error = 0, sum = 0, prevError = 0;
	private double delta = 0.0;

	// ticks per second
	private int setPoint = 0;

	private Encoder encode;

	public Shooter(Encoder encode, double ki, double kp, double kd) {
		this.encode = encode;

		setKI(ki);
		setKP(kp);
		setKD(kd);
	}

	@SuppressWarnings("deprecation")
	public void tick() {

		prevError = error;
		error = (encode.getRaw() - setPoint);
		sum += error / 50;

		delta = (error - prevError) / 50.0;
	}

	public double shoot(double setPoint) {
		double output = kd * delta + kp * error + ki * sum;

		return output;
	}

	public double getError() {
		return error;
	}

	public double getPrevError() {
		return prevError;
	}

	public double getSum() {
		return sum;
	}

	public double getKI() {
		return ki;
	}

	public double getKP() {
		return kp;
	}

	public double getKD() {
		return kd;
	}

	public void setKI(double val) {
		this.ki = val;
	}

	public void setKP(double val) {
		this.kp = val;
	}

	public void setKD(double val) {
		this.kd = val;
	}
}
