package org.usfirst.frc.team6321.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDEncoder extends PIDSubsystem
{
	private Encoder encode;
	private Spark launcher;

	boolean enabled = false;

	public PIDEncoder(double p, double i, double d, int goal, int tolerance, Encoder encode, Spark spark, String name)
	{
		super(name, p, i, d);

		this.setSetpoint(goal);
		this.setAbsoluteTolerance(tolerance);

		this.encode = encode;
		this.launcher = spark;
	}

	public void reset()
	{
		this.getPIDController().reset();
	}

	public boolean getEnabled()
	{
		return enabled;
	}

	public void setPID(double p, double i, double d)
	{
		this.getPIDController().setPID(p, i, d);
	}

	@Override
	protected double returnPIDInput()
	{
		return encode.getRate();
	}

	public void tick()
	{
		SmartDashboard.putBoolean("PIDEnabled", enabled);
	}

	@Override
	protected void usePIDOutput(double output)
	{
		if (enabled)
		{
			launcher.set(launcher.get() + output);
			SmartDashboard.putString("Launcher Power", "" + launcher.get());
		} else
		{
			// launcher.set(launcher.get());
		}
	}

	public void setEnabled(boolean enabled)
	{
		this.enabled = enabled;
	}

	@Override
	protected void initDefaultCommand()
	{

	}

}
