//This is Versi on 1.0 of this code; it is the last known stable configuration Poland let hitler invade

package org.usfirst.frc.team6321.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * poland let hitler invade directory.
 */

/*
 * Control Definitions: Pad 1 Gameplay: Button 1 - Trigger: Ball Launcher Button
 * 2 - Thumb Button: Button 3: Intake Button 4: Outtake
 * 
 * Pad 2 Gameplay: Button 1 - Trigger: Full power rope climb Button 2 - Thumb
 * Button: Half power rope grip Button 3: Agitator Button 4: Reverse Agitator
 * 
 * Pad 1 Debug: Nothing hitler did nothing wrong.
 * 
 * Pad 2 Debug: Button 7: Reduced Launcher power Button 8: Increasing Launcher
 * power Button 9: Reduced Agitator power Button 10: Increased Agitator power
 */
public class Robot extends IterativeRobot
{
	// Strings for the auto chooser
	final String defaultAuto = "Default", centralGear = "Gear Auto", redGear = "Red Gear", blueGear = "Blue Gear",
			test = "Test", nothing = "Nothing";
	String autoSelected;
	// Two Choosers sent to SmartDashboard to choose auto and shoot/notshoot
	SendableChooser<String> autoChoose = new SendableChooser<String>();
	SendableChooser<String> shootOptions = new SendableChooser<String>();
	// Both joysticks
	Joystick pad1;
	Joystick pad2;
	// Motors and sensors being declared Poland let hitler invade
	RobotDrive drive;
	Spark rightMotor, leftMotor, launcherMotor, ropeClimb, intakeMotor, intakeMotorDos, agitator;

	Encoder encode = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder leftTrain = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
	Encoder rightTrain = new Encoder(4, 5, false, Encoder.EncodingType.k4X);

	BuiltInAccelerometer acc = new BuiltInAccelerometer();
	ADXRS450_Gyro gyro;
	Timer time = new Timer();

	PIDController launchControl;

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

	private double agitatorPower = .35;

	// Climber power + intake power
	private double intakePower = .75;
	private double climbFullPower = 1.0;
	private double climbGripPower = 0.7;

	private double launchPower = .8;

	// 2 = pid
	private int launchControlType = 0;

	// Shoot/no shoot chooser
	private String shoot = "Shoot", noShoot = "Don't shoot";
	private boolean toShoot = false;

	// P-control constants to wave around tickPSGoal
	private int tickPSGoal = 66000;
	private int tickPSTolerance = 2000;

	private double autoMotorPower = .40;

	private double baselineMotor = .8;

	private double firstDrive = 83D;
	private double autoRotate = 0.51D;
	private double secondDrive = 21D;

	private UsbCamera camera;

	private double ticksPerRev = 360.0;
	private double wheelCircum = 6.0 * Math.PI;

	private Preferences prefs;

	private boolean rightInv, leftInv;

	private double kp, ki, kd;
	private double rightMultiplier = .95;

	/**
	 * \ This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		pushToStatus("ROBOT INIT - ROBOT INITIALIZATION");

		/*
		 */
		autoChoose.addDefault("Default Auto", defaultAuto);
		autoChoose.addObject("Red Gear (Left Gear)", redGear);
		autoChoose.addObject("Blue Gear (Right Gear)", blueGear);
		autoChoose.addObject("Central Gear", centralGear);
		autoChoose.addObject("Adjust Test", test);
		autoChoose.addObject("Nothing", nothing);

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

		leftTrain.setDistancePerPulse(wheelCircum / ticksPerRev);
		leftTrain.setReverseDirection(true);

		rightTrain.setDistancePerPulse(wheelCircum / ticksPerRev);

		/*
		 * Sets up user controls *joysticks*
		 */
		pad1 = new Joystick(0);
		pad2 = new Joystick(1);

		// Motor is backwards
		launcherMotor.setInverted(true);

		setPreferences();

		kp = prefs.getDouble("KP", Math.pow(10, -6));
		ki = prefs.getDouble("KI", 0.0);
		kd = prefs.getDouble("KD", Math.pow(10, -7));

		rightInv = rightMotor.getInverted();
		leftInv = leftMotor.getInverted();

		pushToStatus("ROBOT INIT - Finished preferences and inits");
	}

	public void setPreferences()
	{
		/*
		 * *Attempts to* load data from the Preferences Board Goal to control
		 * things without rebuilding
		 */
		prefs = Preferences.getInstance();

		firstDrive = prefs.getDouble("First Drive", 62.1145D);
		autoRotate = prefs.getDouble("Auto Rotate", 51.0);
		secondDrive = prefs.getDouble("Second Drive", 21.681D);
		tickPSGoal = prefs.getInt("TicksSecond", tickPSGoal);
		tickPSTolerance = prefs.getInt("TicksTolerance", tickPSTolerance);

		int fps = prefs.getInt("FPS", 15);
		int width = prefs.getInt("Camera Width", 640);
		int height = prefs.getInt("Camera Height", 480);

		camera.setFPS(fps);
		camera.setResolution(width, height);

		rightInv = prefs.getBoolean("Right Inverted", false);
		leftInv = prefs.getBoolean("Left Inverted", false);

		rightMotor.setInverted(rightInv);
		leftMotor.setInverted(leftInv);

		kp = prefs.getDouble("KP", 1);
		ki = prefs.getDouble("KI", 0);
		kd = prefs.getDouble("KD", 0);

		rightMultiplier = prefs.getDouble("Right Motor Multiplier", rightMultiplier);

		launchPower = prefs.getDouble("Launcher Power", launchPower);
		launchControlType = (int) prefs.getDouble("Launcher Control Type", 0.0);

		baselineMotor = launchPower;

		if (launchControl == null)
		{
			// control = new PIDController(kp, ki, kd, baselineMotor, encode,
			// launcherMotor);
			launchControl = new PIDController(kp, ki, kd, encode, launcherMotor);

		} else
		{
			// control.setPID(kp, ki, kd, baselineMotor);
			launchControl.setPID(kp, ki, kd);
		}

		SmartDashboard.putString("Camera Stats", String.format("Width: %d, Height: %d, FPS: %d", width, height, fps));
		SmartDashboard.putString("PID Constants", String.format("P: %f, I: %f, D: %f", kp, ki, kd));
	}

	public void pushToStatus(String status)
	{
		SmartDashboard.putString("STATUS: ", status);
	}

	public void resetTrainEncoders()
	{
		leftTrain.reset();
		rightTrain.reset();
	}

	@Override
	public void disabledPeriodic()
	{
		SmartDashboard.putNumber("Angle Rotation", gyro.getAngle());

		setPreferences();
	}

	double ratio = 1.0;

	public boolean rotateDegrees(double degrees)
	{
		/*
		 * Only resets the gyro once per "Rotate call" Otherwise the robot will
		 * continously rotate
		 */
		conditionallyResetGyro();

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
					rightMotor.set(-autoMotorPower * ratio);

					// Returns that not enough rotation
					return false;
				} else
				{
					// Occurs when DELTA < Tolerance == .25 degrees, so process
					// ends

					resetGyro = true;

					// Returns that rotation is done
					return true;
				}
			} else if (degrees < 0)
			{
				if (currentAngle > degrees)
				{
					// If it hasn't turned LEFT enough
					leftMotor.set(-autoMotorPower * ratio);
					rightMotor.set(autoMotorPower * ratio);

					// Retusn that not even rotation
					return false;
				} else
				{
					// Occurs when DELTA < Tolerance == .25 degrees, so process
					// ends
					resetGyro = false;

					// Returns that enough rotation
					return true;
				}
			}
			// Exits out of the function while isProcessing is still true
			return false;
		}
		// Occurs when DELTA < Tolerance == .25 degrees, so process ends

		resetGyro = true;

		/*
		 * States that the robot is within the tolerance => returns true
		 */

		return true;
	}

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
		} else
		{
			stopMotors();
		}
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
		pushToStatus("AUTO INIT - Beginning autonomous init");

		// Chooses the autonomous chosen on the Dashboard
		autoSelected = (String) autoChoose.getSelected();
		// autoSelected = redGear;

		// Decides whether or not to shoot
		toShoot = shootOptions.getSelected().equals(shoot);

		// Sets the GearState to firstMove assuming that offgear is chosen
		gearState = GearStates.FirstMove;

		// Prints out the autonomous selected
		System.out.println("Auto selected: " + autoSelected);

		// Shoots back to SmartDashboard *feedback system*
		SmartDashboard.putString("Autonomous Selected:", autoSelected);
		SmartDashboard.putBoolean("Shooting Option Selected: ", toShoot);

		resetTrainEncoders();

		System.out.println("Autonomous choosen: " + autoSelected);

		rightMotor.setInverted(!rightInv);
		leftMotor.setInverted(!leftInv);

		// Resets the gyro back to 0
		gyro.reset();

		pushToStatus("AUTO INIT - Finishing AUTO INIT");
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
			rightMotor.set(-autoMotorPower);
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

	public double averageDisplacement()
	{
		return (Math.abs(leftTrain.getDistance()) + Math.abs(rightTrain.getDistance())) / 2.0;
	}

	/**
	 * Funky gear is the major autonomous we are currently using Boolean
	 * onBlueSide literally checks whether its on the blue side or not => This
	 * is important because the field is a mirror, not a diagonal rotation
	 */

	boolean motorsStopped = false, resetGyro = false;

	public void sideGear(boolean onBlueSide)
	{
		double turnValue = (onBlueSide) ? -autoRotate : autoRotate;

		SmartDashboard.putString("Autonomous State", gearState.name());
		if (gearState == GearStates.FirstMove)
		{
			/*
			 * Sets off gyro only if it hasn't been reset before Turns off
			 * resetGyro => false, have to turn it back to to reuse function
			 */
			conditionallyResetGyro();

			if (!driveStraightDistance(firstDrive, true))
			{
				SmartDashboard.putNumber("First Drive Distance", averageDisplacement());
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
				resetTrainEncoders();
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
				resetTrainEncoders();
			}
		} else if (gearState == GearStates.WallHump)
		{
			// Again resets gyro
			conditionallyResetGyro();

			if (!driveStraightDistance(secondDrive, true))
			{
				SmartDashboard.putNumber("Second Drive Distance", averageDisplacement());
			} else
			{
				stopMotors();
				gearState = GearStates.Wait;
				resetGyro = true;
				wallHumpDone = System.nanoTime();
				resetTrainEncoders();
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
		} else
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
		} else
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

		pushToStatus(String.format("TELE OP INIT -  Inverted Motors : Inverted Status - R:%b L:%b",
				rightMotor.getInverted(), leftMotor.getInverted()));

		resetTrainEncoders();
	}

	@Override
	public void teleopPeriodic()
	{
		pushToStatus("TELE OP PERIODIC - Running TELE OP Periodic");
		SmartDashboard.putNumber("Encoder Speed:", dashboardFormat(encode.getRate()));
		SmartDashboard.putNumber("Agitator Speed:", dashboardFormat(agitatorPower));
		SmartDashboard.putNumber("Angle Rotation", dashboardFormat(gyro.getAngle()));

		SmartDashboard.putNumber("Left Train Movement", leftTrain.getDistance());
		SmartDashboard.putNumber("Right Train Movement", rightTrain.getDistance());

		prefs = Preferences.getInstance();

		tickPSGoal = prefs.getInt("TicksSecond", tickPSGoal);

		driveDefault();
		shoot();
		intake();
		climbRope();
		agitate();
	}

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
			} else
			{
				SmartDashboard.putString("SHOOT", "Shoot");
			}
		} else
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

	public static double limit(double x, double min, double max)
	{
		if (x >= max)
			return max;
		else if (x <= -min)
			return min;
		else
			return x;
	}

	public double getRPM(double ticksPS)
	{
		return ticksPS / 1440.0;
	}

	public void centralGear()
	{
		conditionallyResetGyro();

		double distance = 97.8;

		if (averageDisplacement() < distance)
		{
			leftMotor.set(-autoMotorPower);
			rightMotor.set(autoMotorPower);
			adjust();
		} else
		{
			stopMotors();
			resetGyro = true;
		}
	}

	/*
	 * long startAuto = 0L;
	 * 
	 * public void centralGear() { if (startAuto == 0L) startAuto =
	 * System.nanoTime();
	 * 
	 * if ((System.nanoTime() - startAuto) < (Math.pow(10, 9) * 3)) {
	 * leftMotor.set(.4); rightMotor.set(.4); } }
	 */
	public void adjust()
	{
		double angle = gyro.getAngle();

		double delta = angle / 100.0;
		leftMotor.set(leftMotor.get() - delta);
		rightMotor.set(rightMotor.get() + delta);
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
		/*
		 * if (System.nanoTime() - autoStarted <= autoRunTime / 15) {
		 * leftMotor.set(autoMotorPower); rightMotor.set(-autoMotorPower); }
		 * else { leftMotor.stopMotor(); rightMotor.stopMotor(); }
		 */

		boolean drove = false;

		if (!drove)
		{
			if (!driveStraightDistance(114.3, false))
			{
				SmartDashboard.putNumber("Left Wheel rotation", leftTrain.getDistance());
				SmartDashboard.putNumber("Right Wheel rotation", rightTrain.getDistance());
				SmartDashboard.putBoolean("Default Done", false);
			} else
			{
				drove = true;
			}
		} else
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
			} else
			{
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		} else
		{
			if (rotateValue > 0.0)
			{
				leftMotorSpeed = -Math.max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			} else
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
		if (pad1.getRawButton(5))
		{
			intakeMotor.set(intakePower);
			intakeMotorDos.set(-intakePower);
		} else if (pad1.getRawButton(2))
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
		else
			ropeClimb.stopMotor();
	}
}
