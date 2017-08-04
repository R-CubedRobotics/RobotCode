/*
 * Team: 6321
 * Last Updated: August 3rd, 2017
 */
package org.usfirst.frc.team6321.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{
	/*
	 * Enumerates the various strings usable for the SmartDashboard, added in RobotInit(), plus a String for what will be chosen
	 */
	final String defaultAuto = "Default", centralGear = "Gear Auto", redGear = "Red Gear", blueGear = "Blue Gear", test = "Test", nothing = "Nothing";
	String autoSelected;

	/*
	 *  Two Choosers sent to SmartDashboard to choose auto and shoot/notshoot
	 */
	SendableChooser<String> autoChoose = new SendableChooser<String>();
	SendableChooser<String> shootOptions = new SendableChooser<String>();

	// Pad1 and Pad2 declared as the two joysticks for Teleoperated control
	Joystick pad1;
	Joystick pad2;

	// Drive for the drive train
	RobotDrive drive;

	// Numerous motors of the system
	Spark rightMotor, leftMotor, launcherMotor, ropeClimb, intakeMotor, intakeMotorDos, agitator;

	/*
	 * Three different encoders:
	 * Encode: Encoder that measures rotation of the launcher
	 * leftTrain: Encoder that measures rotation of the leftDriveTrain
	 * rigthtTrain: Encoder that measures rotation of the rightDriveTrain
	 * Encode.EncodingType.k4x is the type of Encoder used (measures 4 ticks an actual tick which is internally compensated for).
	 */
	Encoder encode = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder leftTrain = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
	Encoder rightTrain = new Encoder(4, 5, false, Encoder.EncodingType.k4X);

	// Initializes a gyrometer that allows the system to receive feedback on orientation

	ADXRS450_Gyro gyro;

	// Creates PID constants for the PID launcher (launchControl)
	private double kp, ki, kd;

	// Initializes a PID control mechanism to converge to proper launcher shooting speeds 
	PIDController launchControl;

	/*
	 * Sets up a "State" system for the gear autonomous 
	 * Optimal relative to than setting up time based if, else systems
	 */

	private enum GearStates
	{
		FirstMove(), FirstWait(), Rotate(), WallHump(), Wait(), UnHump(), RotateReverse(), PossibleShoot();
	}

	// Creates a simple binary protocol to pass as a parameter to tell the drive functions the direction of movement

	private enum Direction
	{
		FORWARD(), BACKWARD();
	}

	private GearStates gearState;

	private double agitatorPower = .35;

	// Intake power of the 2 intake motors
	private double intakePower = .75;

	// Two climb constants (full power, grip power)
	private double climbFullPower = 1.0;
	private double climbGripPower = 0.7;

	// Default launch power, and launchControlType
	private double launchPower = .8;
	private int launchControlType = 2;

	// Shoot/no shoot chooser
	private String shoot = "Shoot", noShoot = "Don't shoot";
	private boolean toShoot = false;

	// P-control constants to wave around tickPSGoal
	private int tickPSGoal = 66000;
	private int tickPSTolerance = 2000;

	private double autoMotorPower = .40;

	// Primitive time constants for drive times
	private double firstDrive = 83D;
	private double autoRotate = 0.51D;
	private double secondDrive = 21D;

	// Creates a camera object to push to the SmartDashboard
	private UsbCamera camera;

	// Sets some constants to established a full turn and a full turn circumference
	private double ticksPerRev = 360.0;
	private double wheelCircum = 6.0 * Math.PI;

	// Creates a preferences object later initialized in order to get constants from the SmartDashboard
	private Preferences prefs;

	private boolean rightInv, leftInv;

	private double rightMultiplier = .95;

	/**
	 * \ This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		pushToStatus("ROBOT INIT - ROBOT INITIALIZATION");

		/*
		 * Adds the various string options for the autonomous choice board on the SmartDashboard
		 */
		autoChoose.addDefault("Default Auto", defaultAuto);
		autoChoose.addObject("Red Gear (Left Gear)", redGear);
		autoChoose.addObject("Blue Gear (Right Gear)", blueGear);
		autoChoose.addObject("Central Gear", centralGear);
		autoChoose.addObject("Adjust Test", test);
		autoChoose.addObject("Nothing", nothing);

		// Initializes the Camera

		camera = CameraServer.getInstance().startAutomaticCapture(0);

		/*
		 * Sets up Shooting / Not Shooting
		 */
		shootOptions.addDefault("No shooting", noShoot);
		shootOptions.addObject("Shoot", shoot);

		/*
		 * Puts up these options onto the SmartDashboard
		 */
		SmartDashboard.putData("Auto choices", autoChoose);
		SmartDashboard.putData("Shoot Options", shootOptions);

		/*
		 * Initializes the gyro and calibrates the gyro's definition of "straight", a.k.a 0 degree rotation
		 */
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();

		/*
		 * Initializes the motors and sensors required
		 */
		rightMotor = new Spark(0);
		leftMotor = new Spark(1);
		launcherMotor = new Spark(2);
		ropeClimb = new Spark(3);
		intakeMotor = new Spark(4);
		intakeMotorDos = new Spark(5);
		agitator = new Spark(6);
		drive = new RobotDrive(rightMotor, leftMotor);

		// Sets the distance per tick for both the left and right trains (encoder functions)
		leftTrain.setDistancePerPulse(wheelCircum / ticksPerRev);
		leftTrain.setReverseDirection(true);

		rightTrain.setDistancePerPulse(wheelCircum / ticksPerRev);

		// Sets up user controls *joysticks*
		pad1 = new Joystick(0);
		pad2 = new Joystick(1);

		// Launcher motor rotates inwards instead of out, so it needs to be rotated
		launcherMotor.setInverted(true);

		// Initializes the preferences from the SmartDashboard
		setPreferences();

		// Initializes PID constants
		kp = prefs.getDouble("KP", Math.pow(10, -6));
		ki = prefs.getDouble("KI", 0.0);
		kd = prefs.getDouble("KD", Math.pow(10, -7));

		pushToStatus("ROBOT INIT - Finished preferences and inits");
	}

	public void setPreferences()
	{
		// *Attempts to* load data from the Preferences Board Goal to control things without rebuilding
		prefs = Preferences.getInstance();

		//
		firstDrive = prefs.getDouble("First Drive", 62.1145D);
		autoRotate = prefs.getDouble("Auto Rotate", 51.0);
		secondDrive = prefs.getDouble("Second Drive", 21.681D);

		/*
		* Sets the PID launcher goal + the tolerance for the launcher speed
		* Tolerance => if |launcher.getSpeed() - goal| <= tolerance, it's considered on target
		*/
		tickPSGoal = prefs.getInt("TicksSecond", tickPSGoal);
		tickPSTolerance = prefs.getInt("TicksTolerance", tickPSTolerance);

		// Gets the camera goal FPS, width, and height
		int fps = prefs.getInt("FPS", 15);
		int width = prefs.getInt("Camera Width", 640);
		int height = prefs.getInt("Camera Height", 480);

		// Sets the camera fps, width, and height using preference options
		camera.setFPS(fps);
		camera.setResolution(width, height);

		// Gets whether or not we want the right and left motor inverted (leave these options as in on SmartDashboard unless you've fixed the drain train issue)
		rightInv = prefs.getBoolean("Right Inverted", false);
		leftInv = prefs.getBoolean("Left Inverted", false);

		// Sets the inversions based on the preference options
		rightMotor.setInverted(rightInv);
		leftMotor.setInverted(leftInv);

		// Gets PID constants from Prefs
		kp = prefs.getDouble("KP", 1);
		ki = prefs.getDouble("KI", 0);
		kd = prefs.getDouble("KD", 0);

		// Option in order to increase / dampen motor power to compensate for weaker / stronger motor
		rightMultiplier = prefs.getDouble("Right Motor Multiplier", rightMultiplier);

		// Sets a default launcher power in case of flat rate shooting w/o PID Control (shoot type dictated by launchControlType)
		launchPower = prefs.getDouble("Launcher Power", launchPower);
		launchControlType = (int) prefs.getDouble("Launcher Control Type", 0.0);

		// Initializes the launchControl in the case that it hasn't been created yet, else just updates the PID constants
		if (launchControl == null)
		{
			launchControl = new PIDController(kp, ki, kd, encode, launcherMotor);
		}
		else
		{
			launchControl.setPID(kp, ki, kd);
		}

		// Pushes the data from CAMREA + PID options back to SmartDashboard
		SmartDashboard.putString("Camera Stats", String.format("Width: %d, Height: %d, FPS: %d", width, height, fps));
		SmartDashboard.putString("PID Constants", String.format("P: %f, I: %f, D: %f", kp, ki, kd));
	}

	// Helper function to push messages on SmartDashboard (gets irritating to type out)
	public void pushToStatus(String status)
	{
		SmartDashboard.putString("STATUS: ", status);
	}

	/*
	 * Resets the left and right drive train encoders to read 0
	 */
	public void resetTrainEncoders()
	{
		leftTrain.reset();
		rightTrain.reset();
	}

	/*
	 * Describes the actions done by the robot during the period between enables (i.e. any time the robot is disabled)
	 */
	@Override
	public void disabledPeriodic()
	{
		SmartDashboard.putNumber("Angle Rotation", gyro.getAngle());

		setPreferences();
	}

	double ratio = 1.0;

	/**
	 * @param degrees: the degrees the robot should rotate setting 0 to forward (clockwise rotation) 
	 * @return true / false
	 * true: The robot has rotated at least the number of degrees it was told to. "Process" has ended.
	 * false: The robot has not rotated the number of degrees it was told to. The "Process" is still continuing;
	 * 
	 * Note: This works better if you think of this function paired with the return statement as a while loop. 
	 * 		The program continues to "loop" until the gyro location is within the tolerance.
	 * 		
	 * Note 2: The resetGyro calls are meant to allow the function to be reused (the boolean resetGyro is used by conditionallResetGyro() to calibrate location 0)
	 * 
	 * Note 3: Please replace this function with a PID controller for the wheels in the future :) <3 ~Aneesh
	 */
	public boolean rotateDegrees(double degrees)
	{
		// Only resets the gyro once per "Rotate call" Otherwise the robot will continuously rotate
		conditionallyResetGyro();

		double currentAngle = gyro.getAngle();

		// Continues to enter here while DELTA > tolerance == .5 degrees
		if (Math.abs(currentAngle - degrees) > .5)
		{
			double motorPower = autoMotorPower * ratio;

			if (degrees > 0)
			{
				if (currentAngle < degrees)
				{
					// If it hasn't turned RIGHT enough
					driveLeft(motorPower, Direction.FORWARD);
					driveRight(motorPower, Direction.BACKWARD);

					return false;
				}
				else
				{
					resetGyro = true;

					return true;
				}
			}
			else if (degrees < 0)
			{
				if (currentAngle > degrees)
				{
					// If it hasn't turned LEFT enough
					driveLeft(motorPower, Direction.BACKWARD);
					driveRight(motorPower, Direction.FORWARD);

					return false;
				}
				else
				{
					resetGyro = true;

					return true;
				}
			}
			return false;
		}

		resetGyro = true;

		return true;
	}

	/**
	 * Some random auto that really has no use but to run straight at .5 speed for 10 seconds. I'm pretty sure the (drivetrain) orientation is off too.
	 */

	long startTime = 0L;

	public void timeBasedDefaultAuto()
	{
		if (startTime == 0L)
		{
			startTime = System.nanoTime();
		}
		if (System.nanoTime() - startTime < 10000000000L)
		{
			pushToStatus("TIME BASED DEFAULT AUTO - RUNNING STRAIGHT");
			leftMotor.set(.5);
			rightMotor.set(.5);
		}
		else
		{
			stopMotors();
		}
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes using the dashboard. 
	 * The sendable chooser code works with the Java SmartDashboard. 
	 * If you prefer the LabVIEW Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box below the Gyro
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings. 
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit()
	{
		pushToStatus("AUTO INIT - Beginning autonomous init");

		// Chooses the autonomous chosen on the Dashboard
		autoSelected = (String) autoChoose.getSelected();

		// Decides whether or not to shoot
		toShoot = shootOptions.getSelected().equals(shoot);

		// Sets the GearState to firstMove assuming that offgear is chosen
		gearState = GearStates.FirstMove;

		// Prints out the autonomous selected
		System.out.println("Auto selected: " + autoSelected);

		// Shoots back to SmartDashboard *feedback system*
		SmartDashboard.putString("Autonomous Selected:", autoSelected);
		SmartDashboard.putBoolean("Shooting Option Selected: ", toShoot);

		// Any pre-autonomous movement on the drive train shouldn't be counted
		resetTrainEncoders();

		System.out.println("Autonomous choosen: " + autoSelected);

		// Flips the inversions of the system
		rightMotor.setInverted(!rightInv);
		leftMotor.setInverted(!leftInv);

		// Sets the current orientation of the robot to the default
		gyro.reset();

		pushToStatus("AUTO INIT - Finishing AUTO INIT");
	}

	// A boolean to prevent rotation of the bot more than once
	boolean rotated = false;

	// Random test boolean
	boolean driveForwardRefSet = false;

	@Override
	public void autonomousPeriodic()
	{
		pushToStatus("AUTO PERIODIC - Running autonomous periodic");

		SmartDashboard.putNumber("Launcher Speed:", launchPower);
		SmartDashboard.putNumber("Agitator Speed:", agitatorPower);
		SmartDashboard.putNumber("Angle Rotation", gyro.getAngle());

		SmartDashboard.putNumber("Left Motor Power", leftMotor.get());
		SmartDashboard.putNumber("Right Motor Power", rightMotor.get());

		SmartDashboard.putBoolean("Left inverted", leftMotor.getInverted());
		SmartDashboard.putBoolean("Right inverted", rightMotor.getInverted());

		SmartDashboard.putNumber("Left Train Movement", leftTrain.getDistance());
		SmartDashboard.putNumber("Right Train Movement", rightTrain.getDistance());

		switch (autoSelected)
		{
		case defaultAuto:
			timeBasedDefaultAuto();
			break;
		case centralGear:
			centralGear();
			break;
		case nothing:
			doNothingAuto();
			break;
		case redGear:
			// turns right
			sideGear(false);
			break;
		case blueGear:
			// turns left
			sideGear(true);
			break;
		case test:
			testAuto();
			break;
		default:
			// Put default auto code here
			stopMotors();
			break;
		}
	}

	/**
	 * Simple 45 degree rotation
	 */
	public void rotateDebug(boolean x)
	{
		double value = (x) ? -45.0 : 45.0;

		if (!rotated)
		{
			if (rotateDegrees(value))
			{
				SmartDashboard.putBoolean("Turned", true);
				rotated = true;
			}
			else
			{
				SmartDashboard.putBoolean("Turned", false);
			}
		}
		else
		{
			stopMotors();
		}
	}

	/**
	 * Just a function to completely stop both drive train motors
	 */
	public void stopMotors()
	{
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	public double averageDisplacement()
	{
		return (Math.abs(leftTrain.getDistance()) + Math.abs(rightTrain.getDistance())) / 2.0;
	}

	/**
	 * Welcome to my (@author Aneesh's) horrendous state machine.
	 * It works through an enumeration of states (listed at the declarations of this program) that are cycled through as the bot autonomous continues along it's adventure
	 * Only God and I knew what I was coding when I did. Now only God knows.
	 */

	boolean motorsStopped = false, resetGyro = false;

	public void sideGear(boolean onBlueSide)
	{
		double turnValue = (onBlueSide) ? -autoRotate : autoRotate;

		SmartDashboard.putString("Autonomous State", gearState.name());
		if (gearState == GearStates.FirstMove)
		{
			/*
			 * Sets off gyro only if it hasn't been reset before Turns off resetGyro => false, have to turn it back to to reuse function
			 */
			conditionallyResetGyro();

			if (!driveStraightDistance(firstDrive, true))
			{
				SmartDashboard.putNumber("First Drive Distance", averageDisplacement());
			}
			else
			{
				// Puts bot into phase of waiting for motor before rotation
				gearState = GearStates.FirstWait;

				// Stops motors before rotation
				stopMotors();

				// Resets gyro so conditionallyResetGyro() can be used again
				resetGyro = true;

				// Sets off long timer to end wait later on
				waitStarted = System.nanoTime();
				resetTrainEncoders();
			}
		}
		else if (gearState == GearStates.FirstWait)
		{
			if (System.nanoTime() - waitStarted <= 750000000)
			{
				/**
				 * Actually does nothing, that's the point
				 */
			}
			else
			{
				// Sets the gear to rotate mode
				gearState = GearStates.Rotate;
			}

		}
		else if (gearState == GearStates.Rotate)
		{
			if (!motorsStopped)
			{
				// Stops motors once more, but only once
				stopMotors();
				motorsStopped = true;
			}

			if (rotateDegrees(turnValue))
			{
				// Stops motors if we have rotated enough
				stopMotors();
				// Sets mode into full pelvic thrust
				gearState = GearStates.WallHump;
				// Communicates that we have rotateds
				SmartDashboard.putBoolean("Turned", true);

				/*
				 * Sets a rotationCompleted long time to time the forward movement
				 */
				rotationCompleted = System.nanoTime();
			}
			else
			{
				// Else, tells us that it hasn't turned
				SmartDashboard.putBoolean("Turned", false);
				resetTrainEncoders();
			}
		}
		else if (gearState == GearStates.WallHump)
		{
			// Again resets gyro
			conditionallyResetGyro();

			if (!driveStraightDistance(secondDrive, true))
			{
				SmartDashboard.putNumber("Second Drive Distance", averageDisplacement());
			}
			else
			{
				stopMotors();
				gearState = GearStates.Wait;
				resetGyro = true;
				wallHumpDone = System.nanoTime();
				resetTrainEncoders();
			}
		}
		else if (gearState == GearStates.Wait)
		{
			if (System.nanoTime() - wallHumpDone <= 5000000000L)
			{
				// Do nothing waiting for gear pickup

			}
			else
			{
				waitDone = System.nanoTime();
				// gearState = GearStates.UnHump;
				gearState = GearStates.Wait;
			}
		}
		else
		{
			stopMotors();
		}
	}

	// Long timers to time waits and movements
	long waitStarted = 0L, rotationCompleted = 0L, wallHumpDone = 0L, waitDone = 0L, finalOrient = 0L;

	boolean resetDistance = true;

	public boolean driveStraightDistance(double distance, boolean backwards)
	{
		int coeff = (backwards) ? -1 : 1;

		if (resetDistance)
		{
			resetTrainEncoders();
			resetDistance = false;
		}

		if (averageDisplacement() < distance)
		{
			leftMotor.set(coeff * autoMotorPower);
			rightMotor.set(coeff * autoMotorPower);
			adjust();

			return false;
		}
		else
		{
			stopMotors();
			return true;
		}
	}

	@Override
	public void teleopInit()
	{
		pushToStatus("TELE OP INIT - Initializing Tele-Operated systems");

		gyro.reset();
		encode.reset();
		encode.setSamplesToAverage(127);

		encode.setPIDSourceType(PIDSourceType.kRate);

		if (launchControlType == 2)
		{
			launchControl.setContinuous(false);
			launchControl.setAbsoluteTolerance(tickPSTolerance);
			launchControl.setInputRange(0, 80000);
			launchControl.setOutputRange(0, 1);
			launchControl.setSetpoint(tickPSGoal);
			launchControl.enable();
		}

		rightMotor.setInverted(true);
		leftMotor.setInverted(true);

		pushToStatus(String.format("TELE OP INIT -  Inverted Motors : Inverted Status - R:%b L:%b", rightMotor.getInverted(), leftMotor.getInverted()));

		resetTrainEncoders();
	}

	@Override
	public void teleopPeriodic()
	{
		pushToStatus("TELE OP PERIODIC - Running TELE OP Periodic");

		/**
		 * Prints out a myriad of diagnostics that guaranteedly mean absolutely nothing to the driver. 
		 * Mainly useful for debugging at the competition where 90% of the code is written.
		 */
		SmartDashboard.putNumber("Encoder Speed:", dashboardFormat(encode.getRate()));
		SmartDashboard.putNumber("Agitator Speed:", dashboardFormat(agitatorPower));
		SmartDashboard.putNumber("Angle Rotation", dashboardFormat(gyro.getAngle()));
		SmartDashboard.putNumber("Left Train Movement", leftTrain.getDistance());
		SmartDashboard.putNumber("Right Train Movement", rightTrain.getDistance());

		// Does all the functions associated with driving for the robot
		driveDefault();
		shoot();
		intake();
		climbRope();
		agitate();
	}

	/*
	 * Simple tickRate => rotation converter using the fact that a rotation is 1024.0 ticks
	 */
	public double rotations(double rate)
	{
		return rate / 1024.0;
	}

	boolean set = false;

	public void shoot()
	{
		SmartDashboard.putNumber("Launcher Power", -launcherMotor.get());
		SmartDashboard.putNumber("Goal", tickPSGoal);
		if (pad1.getRawButton(1))
		{
			// control.setPID(kp, ki, kd, baselineMotor);
			launchControl.setPID(kp, ki, kd);

			launchControl.setSetpoint(tickPSGoal);

			double diff = launchControl.getError();

			if (Math.abs(diff) >= tickPSTolerance)
			{
				SmartDashboard.putString("SHOOT", "Don't Shoot");
			}
			else
			{
				SmartDashboard.putString("SHOOT", "Shoot");
			}
		}
		else
		{
			launchControl.setPID(kp, ki, kd, 0.0);
			launchControl.setSetpoint(0.0);
			SmartDashboard.putString("SHOOT", "Released");
		}

		SmartDashboard.putNumber("Average Error", launchControl.getError());
	}

	public void conditionallyResetGyro()
	{
		if (resetGyro)
		{
			gyro.reset();
			resetGyro = false;
		}
	}

	/**
	 * Create a basic static utility function (in case this program grows past a single class) that allows you to limit a variable value between an upper bound and lower bound
	 * @param x, min, max
	 * x: basic variable input that you'd like to limit
	 * min: minimum value in the range you'd like
	 * max: maximum value in the range you'd like
	 * 
	 * @return x' (x prime)
	 * x' : a value min <= x' <= max 
	 */
	public static double limit(double x, double min, double max)
	{
		if (x >= max)
			return max;
		else if (x <= -min)
			return min;
		else
			return x;
	}

	public void centralGear()
	{
		conditionallyResetGyro();

		double distance = 97.8;

		if (averageDisplacement() < distance)
		{
			driveLeft(autoMotorPower, Direction.BACKWARD);
			driveRight(autoMotorPower, Direction.BACKWARD);

			leftMotor.set(-autoMotorPower);
			rightMotor.set(autoMotorPower);
			adjust();
		}
		else
		{
			stopMotors();
			resetGyro = true;
		}
	}

	/*
	 * Used in autonomous to compensate for possible mismatched motor strengths (even at the same powers)
	 * Acts as a simple P-control system (if you wanted an official name).
	 * 
	 * Note: Not as efficient as a PID control system, but works absolutely fine for tuning a straight direction
	 */
	public void adjust()
	{
		// Gets the angle of rotation off straight of the robot
		double angle = gyro.getAngle();

		// Adjusts the angle by a factor of 1/100 to use as a P-control system
		double delta = angle / 100.0;

		driveLeft(leftMotor.get() - delta, Direction.BACKWARD);
		driveRight(rightMotor.get() + delta, Direction.BACKWARD);
	}

	/**
	 * @param 
	 * double power: 0.0 - 1.0
	 * Direction direction: FORWARD || BACKWARD
	 */
	public void driveLeft(double power, Direction direction)
	{
		power = Math.abs(power);

		power = (direction == Direction.FORWARD) ? power : -power;

		leftMotor.set(power);
	}

	/**
	 * @param 
	 * double power: 0.0 - 1.0
	 * Direction direction: FORWARD || BACKWARD
	 */
	public void driveRight(double power, Direction direction)
	{
		power = Math.abs(power);

		power = (direction == Direction.FORWARD) ? -power : power;

		rightMotor.set(power);
	}

	public void testAuto()
	{
		rightMotor.set(.4);
		leftMotor.set(.4);
	}

	public void doNothingAuto()
	{
		// Does absolutely nothing
	}

	public void defaultAuto()
	{
		boolean drove = false;

		if (!drove)
		{
			if (!driveStraightDistance(114.3, false))
			{
				SmartDashboard.putNumber("Left Wheel rotation", leftTrain.getDistance());
				SmartDashboard.putNumber("Right Wheel rotation", rightTrain.getDistance());
				SmartDashboard.putBoolean("Default Done", false);
			}
			else
			{
				drove = true;
			}
		}
		else
		{
			SmartDashboard.putBoolean("Default Done", true);
			stopMotors();
			drove = true;
			resetDistance = true;
		}
	}

	public double dashboardFormat(double i)
	{
		return Math.round(100.0 * i) / 100.0;
	}

	public double squareInput(double d)
	{
		return (d > 0) ? (d * d) : (-d * d);
	}

	public void driveDefault()
	{
		double power = (pad1.getThrottle() <= .50) ? squareInput(pad1.getRawAxis(1)) : -squareInput(pad1.getRawAxis(1));
		drive.arcadeDrive(power, pad2.getRawAxis(0));
	}

	public void driveCustom()
	{
		double power = (pad1.getThrottle() <= .50) ? squareInput(pad1.getRawAxis(1)) : -squareInput(pad1.getRawAxis(1));
		arcadeDrive(power, -pad2.getRawAxis(0));
	}

	public void arcadeDrive(double moveValue, double rotateValue)
	{
		double leftMotorSpeed;
		double rightMotorSpeed;

		moveValue = limit(moveValue, -1, 1);
		rotateValue = limit(rotateValue, -1, 1);

		moveValue = (squareInput(moveValue));
		rotateValue = (squareInput(rotateValue));

		if (moveValue > 0.0)
		{
			if (rotateValue > 0.0)
			{
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = Math.max(moveValue, rotateValue);
			}
			else
			{
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		}
		else
		{
			if (rotateValue > 0.0)
			{
				leftMotorSpeed = -Math.max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
			else
			{
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		rightMotorSpeed *= rightMultiplier;

		leftMotor.set(leftMotorSpeed);
		rightMotor.set(rightMotorSpeed);
	}

	public void agitate()
	{
		if (pad1.getRawButton(3))
		{
			agitator.set(agitatorPower);
		}
		else if (pad1.getRawButton(4))
		{
			agitator.set(-agitatorPower);
		}
		else
		{
			agitator.stopMotor();
		}
	}

	public void intake()
	{
		if (pad1.getRawButton(5))
		{
			//  
			intakeMotor.set(intakePower);
			intakeMotorDos.set(-intakePower);
		}
		else if (pad1.getRawButton(2))
		{
			intakeMotor.set(-intakePower);
			intakeMotorDos.set(intakePower);
		}
		else
		{
			// Stops both motors if none of the buttons were pressed
			intakeMotor.stopMotor();
			intakeMotorDos.stopMotor();
		}
	}

	public void climbRope()
	{
		// Full power up the rope
		if (pad2.getRawButton(1))
			ropeClimb.set(climbFullPower);
		// .7 power on the rope
		else if (pad2.getRawButton(2))
			ropeClimb.set(climbGripPower);
		// Stops the motor if none of the buttons were pressed 
		else
			ropeClimb.stopMotor();
	}
}