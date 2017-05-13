//This is Version 1.0 of this code; it is the last known stable configuration

package org.usfirst.frc.team6321.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.hal.PDPJNI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/*
 * Control Definitions: Pad 1 Gameplay: Button 1 - Trigger: Ball Launcher Button
 * 2 - Thumb Button: Button 3: Intake Button 4: Outtake
 * 
 * Pad 2 Gameplay: Button 1 - Trigger: Full power rope climb Button 2 - Thumb
 * Button: Half power rope grip Button 3: Agitator Button 4: Reverse Agitator
 * 
 * Pad 1 Debug: Nothing
 * 
 * Pad 2 Debug: Button 7: Reduced Launcher power Button 8: Increasing Launcher
 * power Button 9: Reduced Agitator power Button 10: Increased Agitator power
 */
public class Robot extends IterativeRobot
{
	// Strings for the auto chooser
	final String defaultAuto = "Default", centralGear = "Gear Auto", redGear = "Red Gear", blueGear = "Blue Gear",
			test = "Test";
	String autoSelected;
	// Two Choosers sent to SmartDashboard to choose auto and shoot/notshoot
	SendableChooser<String> autoChoose = new SendableChooser<String>();
	SendableChooser<String> shootOptions = new SendableChooser<String>();
	// Both joysticks
	Joystick pad1;
	Joystick pad2;
	// Motors and sensors being declared
	RobotDrive drive;
	Spark rightMotor, leftMotor, launcherMotor, ropeClimb, intakeMotor, intakeMotorDos, agitator;
	Encoder encode = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	BuiltInAccelerometer acc = new BuiltInAccelerometer();
	ADXRS450_Gyro gyro;
	Timer time = new Timer();

	/*
	 * Sets up a "State" system for the gear autonomous Much easier than setting
	 * up time based if, else systems
	 */

	private enum GearStates
	{
		FirstMove(), FirstWait(), Rotate(), WallHump(), Wait(), UnHump(), RotateReverse(), PossibleShoot();
	}

	private GearStates gearState;

	DistanceMeter distanceMeter;
	boolean previousButton = false;
	boolean previousButton2 = false;
	boolean currentButton = false;
	boolean currentButton2 = false;
	double speed = 1100;

	// Agitator "debug" button changing constants
	private double agitatorPower = .35;
	private double agitatorPowerDelta = .01;
	private boolean agitatorPowerChangeButtonHeld = false;

	// Climber power + intake power
	private double intakePower = .75;
	private double climbFullPower = 1.0;
	private double climbGripPower = 0.7;

	// Launcher "debug" button changing constants
	private double launchPower = .75; /* .68 is perfect for 12.5V battery */
	private double launchPowerDelta = .01;
	private boolean launchPowerChangeButtonHeld = false;

	// Shoot/no shoot chooser
	private String shoot = "Shoot", noShoot = "Don't shoot";
	private boolean toShoot = false;

	// P-control constants to wave around tickPSGoal
	private int tickPSGoal = 66000;
	private int tickPSTolerance = 2000;

	private double autoMotorPower = .40;

	private long firstDrive = 0L;
	private double autoRotate = 0.51D;
	private long secondDrive = 0L;

	private long autoStarted = 0L;
	private long autoRunTime = 30000000000L;

	private PIDController controller;

	private Shooter shooter;

	private UsbCamera camera;
	private Preferences prefs;

	private double kp, ki, kd;

	/**
	 * \ This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		/*
		 * Initialization that puts all the auto modes onto the screen
		 */
		autoChoose.addDefault("Default Auto", defaultAuto);
		autoChoose.addObject("Red Gear", redGear);
		autoChoose.addObject("Blue Gear", blueGear);
		autoChoose.addObject("Central Gear", centralGear);
		autoChoose.addObject("Adjust Test", test);
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
		 * Resets the gyro
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
		acc = new BuiltInAccelerometer();
		distanceMeter = new DistanceMeter(acc);

		/*
		 * Sets up user controls *joysticks*
		 */
		pad1 = new Joystick(0);
		pad2 = new Joystick(1);

		// Motor is backwards
		launcherMotor.setInverted(true);

		setPreferences();

		kp = prefs.getDouble("KP", 1);
		ki = prefs.getDouble("KI", 0);
		kd = prefs.getDouble("KD", 0);

		controller = new PIDController(kp, ki, kd, encode, launcherMotor);
		controller.setAbsoluteTolerance(tickPSTolerance);
		controller.setOutputRange(-1, 1);
		controller.setInputRange(-80000, 80000);
	}

	public void setPreferences()
	{
		/*
		 * *Attempts to* load data from the Preferences Board Goal to control
		 * things without rebuilding
		 */
		prefs = Preferences.getInstance();

		firstDrive = prefs.getLong("First Drive", 2350000000L);
		autoRotate = prefs.getDouble("Auto Rotate", 51.0);
		secondDrive = prefs.getLong("Second Drive", 800000000L);
		tickPSGoal = prefs.getInt("TicksSecond", tickPSGoal);
		tickPSTolerance = prefs.getInt("TicksTolerance", tickPSTolerance);

		int fps = prefs.getInt("FPS", 15);
		int width = prefs.getInt("Camera Width", 640);
		int height = prefs.getInt("Camera Height", 480);

		camera.setFPS(fps);
		camera.setResolution(width, height);

		kp = prefs.getDouble("KP", 1);
		ki = prefs.getDouble("KI", 0);
		kd = prefs.getDouble("KD", 0);

		if (controller != null)
		{
			controller.setPID(kp, ki, kd);
		}

		SmartDashboard.putString("Camera Stats", String.format("Width: %d, Height: %d, FPS: %d", width, height, fps));
		SmartDashboard.putString("PID Constants", String.format("P: %f, I: %f, D: %f", kp, ki, kd));
	}

	@Override
	public void disabledPeriodic()
	{
		setPreferences();
	}

	boolean gyroReset = false;

	double ratio = 1.0;

	public boolean rotateDegrees(double degrees)
	{
		/*
		 * Only resets the gyro once per "Rotate call" Otherwise the robot will
		 * continously rotate
		 */
		if (!gyroReset)
		{
			gyro.reset();
			gyroReset = true;
		}

		double currentAngle = gyro.getAngle();

		// Continues to enter here while DELTA > tolerance == .25 degrees
		if (Math.abs(currentAngle - degrees) > .5)
		{
			if (degrees > 0)
			{
				if (currentAngle < degrees)
				{
					// If it hasn't turned RIGHT enough
					leftMotor.set(autoMotorPower * ratio);
					rightMotor.set(autoMotorPower * ratio);

					// Returns that not enough rotation
					return false;
				} else
				{
					// Occurs when DELTA < Tolerance == .25 degrees, so process
					// ends
					gyroReset = false;

					// Returns that rotation is done
					return true;
				}
			} else if (degrees < 0)
			{
				if (currentAngle > degrees)
				{
					// If it hasn't turned LEFT enough
					leftMotor.set(-autoMotorPower * ratio);
					rightMotor.set(-autoMotorPower * ratio);

					// Retusn that not even rotation
					return false;
				} else
				{
					// Occurs when DELTA < Tolerance == .25 degrees, so process
					// ends
					gyroReset = false;

					// Returns that enough rotation
					return true;
				}
			}
			// Exits out of the function while isProcessing is still true
			return false;
		}
		// Occurs when DELTA < Tolerance == .25 degrees, so process ends

		gyroReset = false;

		/*
		 * States that the robot is within the tolerance => returns true
		 */

		return true;
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit()
	{
		// Resets the gyro back to 0
		gyro.reset();

		// Chooses the autonomous chosen on the Dashboard
		// autoSelected = (String) autoChoose.getSelected();
		autoSelected = redGear;

		// Decides whether or not to shoot
		toShoot = shootOptions.getSelected().equals(shoot);

		// Sets the GearState to firstMove assuming that offgear is chosen
		gearState = GearStates.FirstMove;

		// Prints out the autonomous selected
		System.out.println("Auto selected: " + autoSelected);

		// Shoots back to SmartDashboard *feedback system*
		SmartDashboard.putString("Autonomous Selected:", autoSelected);
		SmartDashboard.putBoolean("Shooting Option Selected: ", toShoot);

		// Sets a post-declaration start time for autonomous
		autoStarted = System.nanoTime();
	}

	/**
	 * This function is called periodically during
	 */

	/*
	 * Left motor should have positive motor values Right motor should have
	 * negative motor values
	 */

	// A boolean to prevent rotation of the bot more than once
	boolean rotated = false;

	// Random test boolean
	boolean driveForwardRefSet = false;

	@Override
	public void autonomousPeriodic()
	{
		SmartDashboard.putNumber("Launcher Speed:", launchPower);
		SmartDashboard.putNumber("Agitator Speed:", agitatorPower);
		SmartDashboard.putNumber("Angle Rotation", gyro.getAngle());

		SmartDashboard.putNumber("Gyro Rotation", gyro.getAngle());

		switch (autoSelected)
		{
		case defaultAuto:
			defaultAuto();
			break;
		case centralGear:
			centralGear();
			break;
		case redGear:
			// turns right
			doFunkyGear(false);
			break;
		case blueGear:
			// turns left
			doFunkyGear(true);
			break;
		case test:
			if (!driveForwardRefSet)
			{
				gyro.reset();
				driveForwardRefSet = true;
			}
			driveFor3();
			adjust();
			break;
		default:
			// Put default auto code here
			stopMotors();
			break;
		}
	}

	/**
	 * Just a debug function
	 */
	long driveStart = 0L;

	public void driveFor3()
	{
		if (driveStart == 0L)
		{
			driveStart = System.nanoTime();
		}

		if (System.nanoTime() - driveStart < 3000000000L)
		{
			leftMotor.set(-autoMotorPower);
			rightMotor.set(+autoMotorPower);
		} else
		{
			stopMotors();
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
			} else
			{
				SmartDashboard.putBoolean("Turned", false);
			}
		} else
		{
			stopMotors();
		}
	}

	/**
	 * Just a function to completey stop both drive train motors
	 */
	public void stopMotors()
	{
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	/**
	 * Funky gear is the major autonomous we are currently using Boolean
	 * onBlueSide literally checks whether its on the blue side or not => This
	 * is important because the field is a mirror, not a diagonal rotation
	 */

	boolean motorsStopped = false, resetGyro = false;

	// Long timers to time waits and movements
	long waitStarted = 0L, rotationCompleted = 0L, wallHumpDone = 0L, waitDone = 0L, finalOrient = 0L;

	public void doFunkyGear(boolean onBlueSide)
	{
		// Robot has to turn LEFT on blueSide, RIGHT on redSide
		double turnValue = (onBlueSide) ? -autoRotate : autoRotate;
		// Second rotation to point at boiler
		double turn2 = (onBlueSide) ? -65.0 : 65.0;

		// Tells us which autonomous mode we are on
		SmartDashboard.putString("Autonomous State: ", gearState.name());

		if (gearState == GearStates.FirstMove)
		{
			/*
			 * Sets off gyro only if it hasn't been reset before Turns off
			 * resetGyro => false, have to turn it back to to reuse function
			 */
			conditionallyResetGyro();

			if (System.nanoTime() - autoStarted <= firstDrive)
			{
				leftMotor.set(-autoMotorPower);
				rightMotor.set(+autoMotorPower);
				adjust();
			} else
			{
				// Puts bot into phase of waiting for motor before rotation
				gearState = GearStates.FirstWait;
				// Stops motors before rottion
				stopMotors();
				// Resets gyro so conditionallyResetGyro() can be used again
				resetGyro = true;
				// Sets off long timer to end wait later on
				waitStarted = System.nanoTime();
			}
		} else if (gearState == GearStates.FirstWait)
		{
			if (System.nanoTime() - waitStarted <= 750000000)
			{
				/**
				 * Actually does nothing, that's the point
				 */
			} else
			{
				// Sets the gear to rotate mode
				gearState = GearStates.Rotate;
			}

		} else if (gearState == GearStates.Rotate)
		{
			if (!motorsStopped)
			{
				// Stops motors once more, but only once
				stopMotors();
				motorsStopped = true;
			}

			/**
			 * rotateDegrees is a boolean Returns true when rotate has been
			 * completed, false when it hasn't. This if statement continues to
			 * call until it completes Bypasses the Watchdog Thread of the
			 * system to allow us to effectively loop
			 */
			if (rotateDegrees(turnValue))
			{
				// Stops motors if we have rotated enough
				stopMotors();
				// Sets mode into full pelvic thrust
				gearState = GearStates.WallHump;
				// Communicates that we have rotateds
				SmartDashboard.putBoolean("Turned", true);

				/*
				 * Sets a rotationCompleted long time to time the forward
				 * movement
				 */
				rotationCompleted = System.nanoTime();
			} else
			{
				// Else, tells us that it hasn't turned
				SmartDashboard.putBoolean("Turned", false);
			}
		} else if (gearState == GearStates.WallHump)
		{
			// Again resets gyro
			conditionallyResetGyro();

			if (System.nanoTime() - rotationCompleted <= secondDrive)
			{
				leftMotor.set(-autoMotorPower * .9);
				rightMotor.set(+autoMotorPower * .9);
				adjust();
			} else
			{
				stopMotors();
				gearState = GearStates.Wait;
				resetGyro = true;
				wallHumpDone = System.nanoTime();
			}
		} else if (gearState == GearStates.Wait)
		{
			if (System.nanoTime() - wallHumpDone <= 5000000000L)
			{
				// Do nothing waiting for gear pickup

			} else
			{
				waitDone = System.nanoTime();
				// gearState = GearStates.UnHump;
				gearState = GearStates.Wait;
			}
		} else if (gearState == GearStates.UnHump)
		{
			conditionallyResetGyro();

			if (System.nanoTime() - waitDone <= secondDrive)
			{
				leftMotor.set(+autoMotorPower * .9);
				rightMotor.set(-autoMotorPower * .9);
				adjust();
			} else
			{
				stopMotors();
				gearState = GearStates.RotateReverse;
				resetGyro = true;
			}
		} else if (gearState == GearStates.RotateReverse)
		{
			if (rotateDegrees(turn2))
			{
				gearState = GearStates.PossibleShoot;

				finalOrient = System.nanoTime();
			}
		} else if (gearState == GearStates.PossibleShoot)
		{
			if (toShoot)
			{
				// Shoot
				if (System.nanoTime() - finalOrient < 5000000000L)
				{
					launcherMotor.set(launchPower);

					if (adjustLauncherSpeed(tickPSGoal))
					{
						agitator.set(agitatorPower);
					} else
					{
						agitator.stopMotor();
					}
				}
			} else
			{
				// Doing nothing
				stopMotors();
			}
		}
	}

	public void conditionallyResetGyro()
	{
		if (resetGyro)
		{
			gyro.reset();
			resetGyro = false;
		}
	}

	public static double limit(double x)
	{
		if (x >= 1.0)
			return 1.0;
		else if (x <= -1.0)
			return -1.0;
		else
			return x;
	}

	public boolean adjustLauncherSpeed(int rpmGoal)
	{
		double rate = encode.getRate();

		double diff = rpmGoal - rate;
		if (Math.abs(diff) > tickPSTolerance)
		{
			double adjust = ((diff) / 400.0);

			SmartDashboard.putString("Launcher Change", "Increasing Power: " + adjust);
			launcherMotor.set(limit(launcherMotor.get() + adjust));

			return false;
		}

		return true;
	}

	long centralStart = 0L;

	public void centralGear()
	{
		centralStart = (centralStart == 0L) ? System.nanoTime() : centralStart;

		conditionallyResetGyro();

		if (System.nanoTime() - centralStart <= 2000000000L)
		{
			leftMotor.set(-autoMotorPower);
			rightMotor.set(+autoMotorPower);
			adjust();
		} else
		{
			stopMotors();
		}
	}

	public void adjust()
	{
		double angle = gyro.getAngle();

		double delta = angle / 100.0;
		leftMotor.set(leftMotor.get() - delta);
		rightMotor.set(rightMotor.get() - delta);
	}

	public void defaultAuto()
	{
		if (System.nanoTime() - autoStarted <= autoRunTime / 20)
		{
			leftMotor.set(-autoMotorPower);
			rightMotor.set(+autoMotorPower);
		} else
		{
			leftMotor.stopMotor();
			rightMotor.stopMotor();
		}

		if ((System.nanoTime() - autoStarted) <= autoRunTime / 6)
		{
			agitator.set(agitatorPower);
			launcherMotor.set(launchPower);
		} else
		{
			agitator.stopMotor();
			launcherMotor.stopMotor();
		}
	}

	public void ballShoot()
	{
		boolean ballsShot = false;
		if ((System.nanoTime() - autoStarted) <= autoRunTime / 15)
		{
			agitator.set(agitatorPower);
			launcherMotor.set(launchPower);
		} else
		{
			agitator.stopMotor();
			launcherMotor.stopMotor();
			ballsShot = true;
		}

		if (ballsShot)
		{
			if (System.nanoTime() - autoStarted <= autoRunTime)
			{
				leftMotor.set(autoMotorPower);
				rightMotor.set(-autoMotorPower);
			} else
			{
				leftMotor.stopMotor();
				rightMotor.stopMotor();
			}
		}
	}

	/**
	 * Allows the Agitator Speed to be adjusted
	 */

	public void changeAgitatePower()
	{
		if (!agitatorPowerChangeButtonHeld)
		{
			if (pad2.getRawButton(10) && agitatorPower <= .98)
			{
				agitatorPower += agitatorPowerDelta;
				agitatorPowerChangeButtonHeld = true;
			} else if (pad2.getRawButton(9) && agitatorPower >= 0.02)
			{
				agitatorPower -= agitatorPowerDelta;
				agitatorPowerChangeButtonHeld = true;
			}
		} else
		{
			if (!pad2.getRawButton(9) && !pad2.getRawButton(10))
				agitatorPowerChangeButtonHeld = false;
		}
	}

	/**
	 * Allows the Launcher Speed to be adjusted
	 */

	public void changeLauncherPower()
	{

		if (!launchPowerChangeButtonHeld)
		{
			if (pad2.getRawButton(8) && launchPower <= .98)
			{
				launchPower += launchPowerDelta;
				launchPowerChangeButtonHeld = true;
			} else if (pad2.getRawButton(7) && launchPower >= 0.02)
			{
				launchPower -= launchPowerDelta;
				launchPowerChangeButtonHeld = true;
			}
		} else
		{
			if (!pad2.getRawButton(7) && !pad2.getRawButton(8))
				launchPowerChangeButtonHeld = false;
		}
	}

	@Override
	public void teleopInit()
	{
		gyro.reset();
		// controller.enable();
		controller.setSetpoint(0.0);
		controller.setAbsoluteTolerance(tickPSTolerance);
	}

	@Override
	public void teleopPeriodic()
	{
		SmartDashboard.putNumber("Encoder Speed:", dashboardFormat(encode.getRate()));
		SmartDashboard.putNumber("Agitator Speed:", dashboardFormat(agitatorPower));
		SmartDashboard.putNumber("Angle Rotation", dashboardFormat(gyro.getAngle()));

		// SmartDashboard.putNumber("Climber Current",
		// PDPJNI.getPDPTotalCurrent(1));
		SmartDashboard.putNumber("Launcher Setpoint", controller.getSetpoint());
		SmartDashboard.putString("PID Values",
				String.format("Motor Output: %f, Integral: %f", controller.get(), controller.getAvgError()));

		drive();
		// shoot();
		intake();
		climbRope();
		launch();
		agitate();
		changeLauncherPower();
		changeAgitatePower();
	}

	public double dashboardFormat(double i)
	{
		return Math.round(100.0 * i) / 100.0;
	}

	public double squareInput(double d)
	{
		return (d > 0) ? (d * d) : (-d * d);
	}

	public void drive()
	{
		double power = (pad1.getThrottle() <= .50) ? squareInput(pad1.getRawAxis(1)) : -squareInput(pad1.getRawAxis(1));
		drive.arcadeDrive(power, -pad2.getRawAxis(0));
	}

	public void launch()
	{
		if (pad1.getRawButton(1))
		{
			launcherMotor.set(launchPower);
		} else
		{
			launcherMotor.stopMotor();
		}
	}

	public void agitate()
	{
		if (pad1.getRawButton(2))
		{
			agitator.set(agitatorPower);
		} else if (pad1.getRawButton(4))
		{
			agitator.set(-agitatorPower);
		} else
		{
			agitator.stopMotor();
		}
	}

	public void intake()
	{
		if (pad1.getRawButton(4))
		{
			intakeMotor.setInverted(true);
			intakeMotorDos.setInverted(true);
		} else
		{
			intakeMotor.setInverted(false);
			intakeMotorDos.setInverted(false);
		}
		if (pad1.getRawButton(3))
		{
			intakeMotor.set(-intakePower);
			intakeMotorDos.set(intakePower);
		} else
		{
			intakeMotor.stopMotor();
			intakeMotorDos.stopMotor();
		}
	}

	/*
	 * Climbs the rope using 3 buttons
	 */

	public void climbRope()
	{
		if (pad2.getRawButton(1))
			ropeClimb.set(climbFullPower);
		else if (pad2.getRawButton(2))
			ropeClimb.set(climbGripPower);
		else if (pad1.getRawButton(11))
			ropeClimb.set(-climbGripPower);
		else
			ropeClimb.stopMotor();
	}

	public void shoot()
	{
		if (pad1.getRawButton(1))
		{
			controller.setSetpoint(tickPSGoal);
			if (!controller.onTarget())
			{
				// Overspeeding or Underspeeding
				SmartDashboard.putString("SHOOT", "DON'T SHOOT");
			} else
			{
				// Good speed to shoot
				SmartDashboard.putString("SHOOT", "SHOOT");
			}
		} else
		{
			// If not being pressed
			SmartDashboard.putString("SHOOT", "Launcher Released");
			controller.setSetpoint(0.0);
		}
	}
}
