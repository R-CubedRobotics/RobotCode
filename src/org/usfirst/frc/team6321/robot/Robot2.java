//This is Version 1.0 of this code; it is the last known stable configuration

package org.usfirst.frc.team6321.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot2 extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	final String ballShoot = "Ball Auto";
	String autoSelected;
	SendableChooser<String> autoChoose = new SendableChooser<>();
	SendableChooser<Double> powerChoose = new SendableChooser<>();
	Joystick pad1;
	Joystick pad2;
	RobotDrive drive;
	Spark rightMotor, leftMotor, launcherMotor, ropeClimb, intakeMotor, intakeMotorDos, agitator;
	Encoder encode = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	BuiltInAccelerometer acc = new BuiltInAccelerometer();

	DistanceMeter distanceMeter;
	double power = .5;
	// ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	boolean previousButton = false;
	boolean previousButton2 = false;
	boolean currentButton = false;
	boolean currentButton2 = false;
	double speed = 1100;

	private long autoStarted = 0;
	private long autoRunTime = 2000000000L;

	/**
	 * \ This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChoose.addDefault("Default Auto", defaultAuto);
		autoChoose.addObject("My Auto", customAuto);
		autoChoose.addObject("Ball Auto", ballShoot);
		SmartDashboard.putData("Auto choices", autoChoose);

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
		// gyro.calibrate();

		pad1 = new Joystick(0);
		pad2 = new Joystick(1);
		// leftMotor.setInverted(true);
		// rightMotor.setInverted(true);
		// Run this only once to initialize the variables
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
		// gyro.calibrate();
		autoSelected = autoChoose.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		// float angle = (float) (gyro.getAngle() + 90);
		// drive.drive(-0.2, -angle / 30);
		System.out.println("Auto selected: " + autoSelected);

		autoStarted = System.nanoTime();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
			if (System.nanoTime() - autoStarted <= autoRunTime) {
				leftMotor.set(.75);
				rightMotor.set(-.75);
				// keepStraight();
			} else {
				leftMotor.set(0);
				rightMotor.set(0);
			}
			break;
		case ballShoot:

		default:
			// Put default auto code here
			leftMotor.set(0);
			rightMotor.set(0);
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void motorToSpeed() {

	}

	@Override
	public void teleopPeriodic() {
		drive();
		intake();
		climbRope();
		launcher();
		agitate();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		// EVERYTHING IN TELEOP WORKS! IF THE ROBOT DON'T WORK IT'S BECAUSE YOU
		// BROKE IT!!!
		// double currentAngle = gyro.getAngle();
		// System.out.println(currentAngle);

	}

	public double squareInput(double d) {
		return (d > 0) ? (d * d) : (-d * d);
	}

	public void agitate() {
		if (pad1.getRawButton(2)) {
			agitator.set(1.0);
		} else if (pad1.getRawButton(7)) {
			agitator.set(-1.0);
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

		// power = SmartDashboard.getDouble("Input Speed");
		double count = encode.get();
		if (pad1.getRawButton(1)) {
			count -= speed;
			if (count > 0) {
				if (count < 25) {
					power -= 0.0001;
				} else {
					power -= .001;
				}
				// count = encode.get();
				// count -= speed;
				launcherMotor.set(-power);
			}
			if (count < 0) {
				if (count > 25) {
					power += 0.0001;
				} else {
					power += .001;
				}
				// count = en code.get();
				// count -= speed;
				launcherMotor.set(-power);
			}

			encode.reset();
			System.out.println("count == " + count);
			System.out.println("power == " + power);
		} else
			launcherMotor.set(0);

		// SmartDashboard.putDouble("Launcher Speed", power);
	}

	public void intake() {
		if (pad1.getRawButton(3)) {
			intakeMotor.set(-.85);
			intakeMotorDos.set(.85);
		} else if (pad1.getRawButton(4)) {
			intakeMotor.set(.85);
			intakeMotorDos.set(-.85);
		} else {
			intakeMotor.set(0);
			intakeMotorDos.set(0);
		}
	}

	public void climbRope() {
		if (pad2.getRawButton(1))
			ropeClimb.set(1.0);
		else if (pad2.getRawButton(2))
			ropeClimb.set(.70);
		else
			ropeClimb.set(0);
	}

	/*
	 * public void keepStraight() { //gyro.reset();
	 * 
	 * double difference = 0.0;
	 * 
	 * // double currAngle = gyro.getAngle(); //if (currAngle == 0.0)
	 * //difference = rightMotor.get() - leftMotor.get(); //else //difference =
	 * 0;
	 * 
	 * if (currAngle != 0.0) // The following scenario happens when the robot is
	 * veering left; // rightMotor > leftMotor if (currAngle > 0.0 && currAngle
	 * < 180.0) rightMotor.set(rightMotor.get() - 0.01); // this scenario
	 * happens when the robot is veering right; leftMotor // > rightMotor else
	 * if (currAngle < 360.0 && currAngle > 180.0) leftMotor.set(leftMotor.get()
	 * - 0.01);
	 * 
	 * // Keep robot at full power if (leftMotor.get() < 0.99 &&
	 * rightMotor.get() < 0.99) { leftMotor.set(1.0 - difference);
	 * rightMotor.set(1.0); } else difference = leftMotor.getSpeed() -
	 * rightMotor.getSpeed(); }
	 */
}