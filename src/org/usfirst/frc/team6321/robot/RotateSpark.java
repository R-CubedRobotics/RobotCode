package org.usfirst.frc.team6321.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class RotateSpark
{

	static boolean isAtDesiredPos = false;

	static boolean shouldResetGyro = true;

	static double currentAngle = 0.0;

	public static boolean rotate(Spark motor, Gyro gyro, double movementSpeed, double degrees, double tolerance)
	{
		// Resets the gyro only at the beginning of a rotate sequence
		if (shouldResetGyro)
		{
			// Sets gyro to 9
			gyro.reset();
			// Makes sure the Gyro doesn't reset again
			shouldResetGyro = false;
		}

		// Updates the current angle
		currentAngle = gyro.getAngle();

		// Only goes continues if currentAngle is off the goal by more than the
		// tolerance
		if (Math.abs(currentAngle - degrees) > tolerance)
		{
			// If its less rotated, it'll turn right / left if needed
			if (motor.getInverted())
			{
				/*
				 * Motor is backward If currentAngle < degrees, rotates !right =
				 * left, else right
				 */
				double power = (currentAngle < degrees) ? -movementSpeed : +movementSpeed;
				motor.set(power);

			} else
			{
				/*
				 * Motor is forward If currentAngle < degrees, rotates !left =
				 * right, else left
				 */
				double power = (currentAngle < degrees) ? +movementSpeed : -movementSpeed;
				motor.set(power);
			}

			// Tells that the process is not done yet
			return false;
		} else
		{
			// Stops the motor so it doesn't continue moving
			motor.stopMotor();

			// Makes sure that the code can be used again
			shouldResetGyro = true;

			// Tells that the process is done and that it can stop
			return true;
		}
	}

	public static double getAngle()
	{
		return currentAngle;
	}

	public static boolean rotateWheels(Spark motor1, Spark motor2, Gyro gyro, double movementSpeed, double degrees,
			double tolerance)
	{
		// Resets the gyro only at the beginning of a rotate sequence
		if (shouldResetGyro)
		{
			// Sets gyro to 9
			gyro.reset();
			// Makes sure the Gyro doesn't reset again
			shouldResetGyro = false;
		}

		// Updates the current angle
		currentAngle = gyro.getAngle();

		// Only goes continues if currentAngle is off the goal by more than the
		// tolerance
		if (Math.abs(currentAngle - degrees) > tolerance)
		{
			// If its less rotated, it'll turn right / left if needed
			if (motor1.getInverted())
			{
				/*
				 * Motor is backward If currentAngle < degrees, rotates !right =
				 * left, else right
				 */
				double power = (currentAngle < degrees) ? -movementSpeed : +movementSpeed;
				motor1.set(power);

			} else
			{
				/*
				 * Motor is forward If currentAngle < degrees, rotates !left =
				 * right, else left
				 */
				double power = (currentAngle < degrees) ? +movementSpeed : -movementSpeed;
				motor1.set(power);
			}

			if (motor2.getInverted())
			{
				/*
				 * Motor is backward If currentAngle < degrees, rotates !right =
				 * left, else right
				 */
				double power = (currentAngle < degrees) ? -movementSpeed : +movementSpeed;
				motor2.set(power);

			} else
			{
				/*
				 * Motor is forward If currentAngle < degrees, rotates !left =
				 * right, else left
				 */
				double power = (currentAngle < degrees) ? +movementSpeed : -movementSpeed;
				motor2.set(power);
			}
			// Tells that the process is not done yet
			return false;
		} else
		{
			// Stops the motor so it doesn't continue moving
			motor1.stopMotor();
			motor2.stopMotor();
			// Makes sure that the code can be used again
			shouldResetGyro = true;

			// Tells that the process is done and that it can stop
			return true;
		}	}
}
