//This is Version 1.0 of this code; it is the last known stable configuration

package org.usfirst.frc.team6321.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default", customAuto = "My Auto", ballShoot = "Ball Auto", carryGear = "Gear Auto",BlueshootingAuto = "BlueShootingAuto",RedshootingAuto = "notBlueshootingAuto";
	String autoSelected;
	SendableChooser<String> autoChoose = new SendableChooser<String>();
	Joystick pad1;
	Joystick pad2;
	RobotDrive drive;
	Spark rightMotor, leftMotor, launcherMotor, ropeClimb, intakeMotor, intakeMotorDos, agitator;
	Encoder encode = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	BuiltInAccelerometer acc = new BuiltInAccelerometer();
	PIDController pidControl;
	ADXRS450_Gyro gyro;
	Timer time = new Timer();

	DistanceMeter distanceMeter;
	boolean previousButton = false;
	boolean previousButton2 = false;
	boolean currentButton = false;
	boolean currentButton2 = false;
	double speed = 1100;

	private double agitatorPower = .58;
	private double agitatorPowerDelta = .01;
	private boolean agitatorPowerChangeButtonHeld = false;

	private double intakePower = 1.0;
	private double climbFullPower = 1.0;
	private double climbGripPower = 0.7;

	private double launchPower = .64; /* .68 is perfect for 12.5V battery */
	private double launchPowerDelta = .01;
	private boolean launchPowerChangeButtonHeld = false;
	

	private double autoMotorPower = .75;
	private double ShootingAutoPowerDrivePower =.50;

	private long prevTime, nowTime;
	private int prevRotation, nowRotation;

	private long autoStarted = 0L;
	private long autoRunTime = 30000000000L;

	private Shooter shooter;

	/**
	 * \ This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChoose.addDefault("Default Auto", defaultAuto);
		autoChoose.addObject("My Auto", customAuto);
		autoChoose.addObject("Ball Auto", ballShoot);
		autoChoose.addObject("Gear Auto", carryGear);
		autoChoose.addObject("RedShootingAuto", BlueshootingAuto);
		autoChoose.addObject("notBlueShootingAuto", RedshootingAuto);
		SmartDashboard.putData("Auto choices", autoChoose);

		gyro = new ADXRS450_Gyro();
		gyro.calibrate();

		SmartDashboard.putNumber("Launcher Speed:", launchPower);
		SmartDashboard.putNumber("Agitator Speed:", agitatorPower);
		SmartDashboard.putNumber("Angle Rotation", gyro.getAngle());

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

		pad1 = new Joystick(0);
		pad2 = new Joystick(1);

		Preferences prefs = Preferences.getInstance();

		pidControl = new PIDController(prefs.getDouble("KP Constant", 1.0), prefs.getDouble("KI Constant", 0.0),
				prefs.getDouble("KD Constant", 0.0), encode, launcherMotor);
		pidControl.setPercentTolerance(1.0);

	}

	private double setPoint;

	public void rotateDegrees(double degrees) {
		setPoint = gyro.getAngle();
		double startAngle = (degrees > 0) ? setPoint : setPoint + 360;

		while (Math.abs(startAngle - degrees) > 1.0) {
			if ((setPoint - startAngle) < degrees) {
				leftMotor.set(autoMotorPower);
				rightMotor.set(-autoMotorPower);
			} else if ((setPoint - startAngle) > degrees) {
				leftMotor.set(-autoMotorPower);
				rightMotor.set(autoMotorPower);
			}
		}
	}

	/**
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
	public void autonomousInit() {
		autoSelected = (String) autoChoose.getSelected();
		System.out.println("Auto selected: " + autoSelected);

		SmartDashboard.putString("Autonomous Selected:", autoSelected);

		gyro.reset();

		autoStarted = System.nanoTime();
	}

	/**
	 * This function is called periodically during autonomous
	 */

	/*
	 * Left motor should have positive motor values Right motor should have
	 * negative motor values
	 */

	@Override
	public void autonomousPeriodic() {
		changeLauncherPower();
		changeAgitatePower();
	
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
			defaultAuto();
			break;
		case BlueshootingAuto:
			shoot();
			move(true);
			break;
		case RedshootingAuto:
			  shoot();
			  move(false);
			  break;
		case ballShoot:
			ballShoot();
			break;
		case carryGear:
			gearRunningAuto();
			break;
		default:
			// Put default auto code here
			doNothing();
			break;
		}
	}

	public void doNothing() {
		leftMotor.set(0);
		rightMotor.set(0);
	}

	public void gearRunningAuto() {
		long startTime = System.nanoTime();

		leftMotor.setInverted(true);
		rightMotor.setInverted(true);

		if (System.nanoTime() - startTime < autoRunTime / 25) {
			leftMotor.set(autoMotorPower/2);
			rightMotor.set(-autoMotorPower/2 *1.04);

			/*adjust();*/
		}

		leftMotor.setInverted(false);
		rightMotor.setInverted(false);
	}

	private double preAngle, nowAngle;

	public void adjust() {
		preAngle = nowAngle;

		nowAngle = gyro.getAngle();

		// now - previous
		double difference = nowAngle - preAngle;

		if (preAngle < 360.0 && nowAngle > 0) {
			difference += 360.0;
		} else if (preAngle > 0.0 && nowAngle < 360.0) {
			difference -= 360.0;
		}

		if (Math.abs(difference) > 1.0) {
			// Wheels drifted right
			if (difference > 0) {
				rightMotor.set(rightMotor.get() + .05);
				leftMotor.set(leftMotor.get() - .05);
			}
			// Wheels drifted left
			else if (difference < 0) {
				rightMotor.set(rightMotor.get() - .05);
				leftMotor.set(leftMotor.get() + .05);
			}
		}
	}

	public void defaultAuto() {
		// rotateDegrees(90);

		if (System.nanoTime() - autoStarted <= autoRunTime / 15) {
			leftMotor.set(autoMotorPower);
			rightMotor.set(-autoMotorPower);
		} else {
			leftMotor.set(0);
			rightMotor.set(0);
		}
	}

	public void ballShoot() {
		boolean ballsShot = false;
		if ((System.nanoTime() - autoStarted) <= autoRunTime / 15) {
			agitator.set(agitatorPower);
			launcherMotor.set(-launchPower);
		} else {
			agitator.set(0);
			launcherMotor.set(0);
			ballsShot = true;
		}

		if (ballsShot) {
			if (System.nanoTime() - autoStarted <= autoRunTime) {
				leftMotor.set(autoMotorPower);
				rightMotor.set(-autoMotorPower);
			} else {
				leftMotor.set(0);
				rightMotor.set(0);
			}
		}
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void changeAgitatePower() {
		if (!agitatorPowerChangeButtonHeld) {
			if (pad2.getRawButton(9) && agitatorPower <= .98) {
				agitatorPower += agitatorPowerDelta;
				agitatorPowerChangeButtonHeld = true;
			} else if (pad2.getRawButton(10) && agitatorPower >= 0.00) {
				agitatorPower -= agitatorPowerDelta;
				agitatorPowerChangeButtonHeld = true;
			}
		} else {
			if (!pad2.getRawButton(9) && !pad2.getRawButton(10))
				agitatorPowerChangeButtonHeld = false;
		}
	}

	public void changeLauncherPower() {
		if (!launchPowerChangeButtonHeld) {
			if (pad2.getRawButton(7) && launchPower <= .98) {
				launchPower += launchPowerDelta;
				launchPowerChangeButtonHeld = true;
			} else if (pad2.getRawButton(8) && launchPower >= 0.00) {
				launchPower -= launchPowerDelta;
				launchPowerChangeButtonHeld = true;
			}
		} else {
			if (!pad2.getRawButton(7) && !pad2.getRawButton(8))
				launchPowerChangeButtonHeld = false;
		}
	}

	public double getRPS() {
		double changeInRotation = (nowRotation > prevRotation) ? nowRotation - prevRotation
				: nowRotation - prevRotation + 256;
		double delta = (nowTime - prevTime);
		return changeInRotation / delta;
	}

	public void launcherTick() {
		prevTime = nowTime;
		prevRotation = nowRotation;

		nowTime = System.nanoTime();
		nowRotation = launcherMotor.getRaw();
	}

	@Override
	public void teleopInit() {
		gyro.reset();
	}

	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("Encoder Speed:", dashboardFormat(encode.getRate()));
		SmartDashboard.putNumber("Agitator Speed:", dashboardFormat(agitatorPower));
		SmartDashboard.putNumber("Angle Rotation", gyro.getAngle());

		drive();
		intake();
		climbRope();
		launcher();
		agitate();
		launcherTick();
		changeLauncherPower();
		changeAgitatePower();
	}

	public double dashboardFormat(double i) {
		return Math.round(100.0 * i) / 100.0;
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

	}

	public double squareInput(double d) {
		return (d > 0) ? (d * d) : (-d * d);
	}

	public void agitate() {
		if (pad2.getRawButton(3)) {
			agitator.set(agitatorPower);
		} else if (pad2.getRawButton(4)) {
			agitator.set(-agitatorPower);
		} else {
			agitator.set(0);
		}
	}

	public void drive() {
		double power = (pad1.getThrottle() <= .50) ? squareInput(pad1.getRawAxis(1)) : -squareInput(pad1.getRawAxis(1));
		drive.arcadeDrive(power, -pad2.getRawAxis(0));
	}

	public void launcher() {
		// 1024 ticks/ motor rations

		double count = encode.get();
		if (pad1.getRawButton(1)) {
			launcherMotor.set(-launchPower);
		} else {
			launcherMotor.set(0);
		}
	}

	public void intake() {
		if (pad1.getRawButton(4)) {
			intakeMotor.setInverted(true);
			intakeMotorDos.setInverted(true);
		} else {
			intakeMotor.setInverted(false);
			intakeMotorDos.setInverted(false);
		}
		if (pad1.getRawButton(3)) {
			intakeMotor.set(-intakePower);
			intakeMotorDos.set(intakePower);
		} else {
			intakeMotor.set(0);
			intakeMotorDos.set(0);
		}
	}

	public void climbRope() {
		if (pad2.getRawButton(1))
			ropeClimb.set(climbFullPower);
		else if (pad2.getRawButton(2))
			ropeClimb.set(climbGripPower);
		else if (pad1.getRawButton(11))
			ropeClimb.set(-1*climbGripPower);
		else
			ropeClimb.set(0);
	}
	public void shoot()
	{
		double t = Timer.getMatchTime();
		if(t < 1){
			launcherMotor.set(launchPower);
		}
        else if(t > 1 && t < 8)//it will only run when time is in between 1 and 8
		{
			agitator.set(agitatorPower);
		}
		else if(t > 8){
			launcherMotor.stopMotor();
			agitator.stopMotor();
		}
		
	}
	
	public void move(boolean isBlue)
	{	
	 double x = Timer.getMatchTime();
	 double ang = gyro.getAngle();
	 
	 
	 if(Math.abs(ang) < 45 && x > 8 && x < 10)
	 {
		 if(isBlue == true)
		 {
			  drive.arcadeDrive(0,ShootingAutoPowerDrivePower);
		 }
		 else
		 {
			 drive.arcadeDrive(0, -ShootingAutoPowerDrivePower);
		 }

		 
	 }
	 else if(x > 10 && x > 12)
	 { 
		 drive.arcadeDrive(autoMotorPower, 0); 
	 }
	}
}


