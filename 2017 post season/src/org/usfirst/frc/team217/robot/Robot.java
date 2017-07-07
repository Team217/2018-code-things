package org.usfirst.frc.team217.robot;

import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.wpilibj.*;
//import QuickPID.PID;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import java.lang.Math;

/**
 * x The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

// Hood angle is 745

public class Robot extends IterativeRobot {
	final String redSide = "Red Auto";
	final String blueSide = "Blue Auto";
	final String ballAuton = "40 Ball Auto";
	final String gearAuton = "Gear and Shoot Auto";
	final String debug = "Debug";
	
	final String left = "Left";
	final String center = "Center";
	final String right = "Right";

	String sideSelected, autoSelected, positionSelected;
	// boolean turretFlip=true;

	double flywheelPID;

	boolean turretFlip = true;

	// Joysticks
	Joystick oper, driver;
	Preferences prefs;

	// Joystick Variables
	final int buttonSquare = 1;
	final int buttonX = 2;
	final int buttonCircle = 3;
	final int buttonTriangle = 4;
	final int leftBumper = 5;
	final int rightBumper = 6;
	final int leftTrigger = 7;
	final int rightTrigger = 8;
	final int buttonShare = 9;
	final int buttonOption = 10;
	final int leftAnalog = 11;
	final int rightAnalog = 12;
	final int buttonPS = 13;
	final int touchpad = 14;

	// PID flyWheelPID;
	double wheelRPM, hoodAngle, turning, flyWheelSpeed, wheelOfDoomSpeed;
	double flyP, flyI, flyD, flyF;
	double visionP, visionI, visionD;
	double vPID, flyPID;
	double gearArmP, gearArmI, gearArmD;
	double hoodP, hoodI, hoodD;	
	double driveP, driveI, driveD;
	double gyroP, gyroI, gyroD;
	double turretP, turretI, turretD;
	
	// Drive Motors
	// left motor is flipped
	CANTalon rightMaster, rightSlave, leftMaster, leftSlave;

	// Shooting Motors
	CANTalon flyWheelMaster, flyWheelSlave, hoodMotor, turretMotor, lifterMotor, kickerMotor, ballIntakeMotor,
			wheelOfDoomMotor;

	// Climbing Motors
	CANTalon climberMaster, climberSlave;

	// Gear Motors
	CANTalon gearArmMotor, gearIntakeMotor;

	// Solenoids
	Solenoid backWheelSolenoid, frontWheelSolenoid, ballIntakeSolenoid;

	// Gyro
	ADXRS450_Gyro horzGyro;

	DigitalInput resetSwitch;

	// Compressor
	Compressor compressor;

	Preferences pref;

	AnalogInput hoodEncoder;

	NetworkTable table, visionTable;

	PID flyWheelPID = new PID(0.0025, 0, 0.015);
	PID visionPID = new PID(0.01, 0.0001, 0.01);
	PID gearArmPID = new PID(0.0005, 0.0000035, 0.001);
	PID hoodPID = new PID(0.005, 0.0002, 0.001);
	PID driveRPID = new PID(0.0003, 0, .0012);
	PID driveLPID = new PID(0.0003, 0, .0012);
	PID gyroPID = new PID(.075, 0 , 0);
	
	double distance, turnValue;
	double driveSpeedLeft, driveSpeedRight, turnSpeed;
	
	boolean shoot;
	
	boolean camNum = false, autonEncReset = true;
	// double flyWheelRPM = -4400;

	enum BallAuton {
		forward, turn, tacticalReload, visionTrack, stopAndShoot
	};

	BallAuton shootingAuton;

	enum GearAuton {
		forward, // same for each side of the field, different for positions
		turn, // same for each side of the field, different for positions
		drop, // same
		reverse,
		align, // different for all
		shoot, // different for positions
		dontMove// same for all
	}

	GearAuton gearShootAuton;

	enum DriveForward {
		drive, stop
	}

	DriveForward driveAuto;

	
	
	// shootingAuto variables
	// turn for blue = -90
	// turn for red = 87
	// turret angle for blue = -2590
	// turret angle for red = -2530

	double climbSpeed;
	int autoRPM;// change based on field, might have to be different for each
				// side
	int forward1;// change based on field, might have to be different for each
					// side

	// gearAuto variables
	// turn for blue = 82
	// turn for red =

	int forward;
	int reverse;
	
	double gearCarry, gearPlace, gearDrop;

	// will be used in both shootingAuton and gearAuton
	double turretAngle; // change based on field
	double turnAngle;// change based on field

	// PID variables
	double cumulativePIDError = 0;
	// PID P val for going straight
	final double PSTRAIGHT = .00217;
	// PID P val for turning
	final double PTURN = .02;
	// PID I value for turning
	final double ITURN = .0002;

	SendableChooser<String> side = new SendableChooser<>();
	SendableChooser<String> auton = new SendableChooser<>();
	SendableChooser<String> position = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		// gearTime = new Timer();

		// Smart Dashboard
		side.addDefault("Red Alliance", redSide);
		side.addObject("Blue Alliance", blueSide);
		SmartDashboard.putData("Auton Side Selection", side);

		auton.addDefault("40 Ball", ballAuton);
		auton.addObject("10 Ball Gear Auto", gearAuton);
		auton.addObject("Debug", debug);
		SmartDashboard.putData("Auton Selection", auton);

		position.addDefault("Left", left);
		position.addObject("Center", center);
		position.addObject("Right", right);
		SmartDashboard.putData("Position Selection", position);

		pref = Preferences.getInstance();

		hoodEncoder = new AnalogInput(1);

		// Gear Manipulator
		gearArmMotor = new CANTalon(8);
		gearArmMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

		gearIntakeMotor = new CANTalon(7);

		// Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);

		resetSwitch = new DigitalInput(1);

		// Drive Motors
		rightMaster = new CANTalon(15);
		rightMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);

		rightSlave = new CANTalon(14);
		rightSlave.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightSlave.changeControlMode(TalonControlMode.Follower);
		rightSlave.set(15);

		leftMaster = new CANTalon(0);
		leftMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);

		leftSlave = new CANTalon(1);
		leftSlave.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftSlave.changeControlMode(TalonControlMode.Follower);
		leftSlave.set(0);

		// Shooting Motors
		flyWheelMaster = new CANTalon(10);
		flyWheelMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		flyWheelMaster.reverseSensor(false);
		flyWheelSlave = new CANTalon(11);
		flyWheelSlave.changeControlMode(TalonControlMode.Follower);
		flyWheelSlave.set(10);

		hoodMotor = new CANTalon(9);
		turretMotor = new CANTalon(6);
		turretMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		lifterMotor = new CANTalon(5);
		kickerMotor = new CANTalon(4);
		ballIntakeMotor = new CANTalon(13);
		wheelOfDoomMotor = new CANTalon(12);

		// flyWheel Settings
		flyWheelMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		flyWheelMaster.configPeakOutputVoltage(+12.0f, -12.0f);

		// Climbing Motors
		climberMaster = new CANTalon(2);
		climberSlave = new CANTalon(3);
		climberSlave.changeControlMode(TalonControlMode.Follower);
		climberSlave.set(2);

		// Solenoids
		frontWheelSolenoid = new Solenoid(2);
		backWheelSolenoid = new Solenoid(1);
		ballIntakeSolenoid = new Solenoid(3);

		// Gyro
		horzGyro = new ADXRS450_Gyro();

		table = NetworkTable.getTable("SmartDashboard");

		table.putBoolean("Camera_Type", camNum);

		visionTable = NetworkTable.getTable("Vision");
		table = NetworkTable.getTable("Vision");

		UsbCamera camera1;
		camera1 = CameraServer.getInstance().startAutomaticCapture();
		camera1.setResolution(320, 240);
		camera1.setFPS(30);
		horzGyro.calibrate();
		smartDash();

		rightMaster.setEncPosition(0);
		rightSlave.setEncPosition(0);
		leftMaster.setEncPosition(0);
		leftSlave.setEncPosition(0);
		turretMotor.setEncPosition(0);
		
		gearCarry = 50;
		gearPlace = 435;
		gearDrop = 1000;
	}


	@Override
	public void autonomousInit() {
		encReset();
		smartDash();
		sideSelected = (String) side.getSelected();
		autoSelected = (String) auton.getSelected();
		positionSelected = (String) position.getSelected();

		switch (autoSelected) {
		case debug:
			gyroPID.Reset();
			break;
		case gearAuton:
			gearAutoInit(sideSelected.equals(blueSide));
			break;
		case ballAuton:
			// call in function
			// pass in boolean param
			// blue=true
			shootingAutoInit(sideSelected.equals(blueSide));
			break;
		}

	}

	void encReset() {
		rightMaster.setEncPosition(0);
		rightSlave.setEncPosition(0);
		leftMaster.setEncPosition(0);
		leftSlave.setEncPosition(0);
		turretMotor.setEncPosition(0);
		ballIntakeMotor.setEncPosition(0);
		horzGyro.reset();
	}
	
	void debug() {
		driveSpeedLeft = driveLPID.GetOutput(leftMaster.getEncPosition(), -distance);
		driveSpeedRight = driveRPID.GetOutput(rightMaster.getEncPosition(), distance);
		turnSpeed = gyroPID.GetOutput(horzGyro.getAngle(), turnValue);
		
		frontWheelSolenoid.set(true);
		backWheelSolenoid.set(false);
		
		
		leftMaster.set(driveSpeedLeft - (turnSpeed * 2.5));
		
		if (absVal(leftMaster.getEncPosition() - distance) < 200) {
			driveLPID.Reset();
		}
		
		rightMaster.set(driveSpeedRight - (turnSpeed * 2.5));
		
		if (absVal(absVal(rightMaster.getEncPosition()) - distance) < 200) {
			driveRPID.Reset();
		}
		
		if(absVal(horzGyro.getAngle()) < .5){
			gyroPID.Reset();
		}
		
		/*if (absVal(horzGyro.getAngle() - turnValue) < 2) {
			gyroPID.Reset();
		}
		
		frontWheelSolenoid.set(true);
		backWheelSolenoid.set(true);
		double turnSpeed = gyroPID.GetOutput(horzGyro.getAngle(), turnValue);
		leftMaster.set(-turnSpeed);
		rightMaster.set(-turnSpeed);
		*/
		
	}

	// when boolean blue = true, the bot is on the blue side
	// when boolean blue = false, bot is on red side
	void shootingAutoInit(boolean blue) {
		wheelRPM = 4110;
		flyWheelPID.SetP(.0025);
		flyWheelPID.SetI(0);
		flyWheelPID.SetD(0.015);
		//wheelOfDoomMotor at 45
		
		if (blue) {
			turnAngle = -90;
			turretAngle = -2590;
		}

		else {
			turnAngle = 87;
			turretAngle = -2530;
			climbSpeed = .65;
		}

		autoRPM = -4450; // Change RPM through testing, increase value for both
							// high and long shots, use hood vale
		forward1 = 4400;
		turretMotor.setEncPosition(0);
		frontWheelSolenoid.set(true);
		backWheelSolenoid.set(false);
		shootingAuton = BallAuton.forward;
	}

	void gearAutoInit(boolean blue) {

		horzGyro.reset();
		leftMaster.setEncPosition(0);
		rightMaster.setEncPosition(0);
		gearArmMotor.setEncPosition(0);
		
		frontWheelSolenoid.set(false);
		backWheelSolenoid.set(false);
		
		hoodAngle = 770;
		reverse = 700;
		wheelRPM = -4110;
	
		
		gearShootAuton = GearAuton.forward;
		
		
		

		switch (positionSelected) {
		case left:
			turnAngle = 82;
			if (blue) {
				forward = 6500;
			} else {
				shoot = true;
				forward = 7570;

			}
			break;

		case center:
			turnAngle = 0;
			forward = -3500
					;
			if (blue) {
				turretAngle = -1600;

			} else {
				turretAngle = 8000;

			}
			break;

		case right:
			turnAngle = -82;
			if (blue) {
				forward = 5650;
				turretAngle = 2500;

			} else {

			}
			break;

		}

	}

	/*
	 * Gear stuff, pos 1: Different: Turn- Blue: 82 Red: -56 Forward: Left:
	 * Blue: -6400 Red: -7470 Right: Blue: 6600 Red: 7670 Timers: Red has timer?
	 * Whaaaaa Same: Resetting enc positions
	 * 
	 * Gear stuff, pos 2: Turret: Blue: 2200 Red: 8000 Same: Solenoid both false
	 * Enc Position reset Forward: Left: Blue: -3370 Red: -3370 Right: Blue:
	 * 3470 Red: 3470
	 * 
	 */

	/**
	 * This function is called periodically during autonomous
	 */

	@Override
	public void autonomousPeriodic() {
		smartDash();
		if (autoSelected.equals(gearAuton)) {
			gearAuto();
		}
		if (autoSelected.equals(ballAuton)) {
			shootingAuto();
		}
		if (autoSelected.equals(debug)) {
			debug();
		}
	}

	void gearAuto() {
		
		//1400 is intake
		switch (gearShootAuton) {
		case forward:
			autoHood(hoodAngle);
			autoTurret(turretAngle);
			autoGearArm(gearPlace);
			autoShoot(wheelRPM);
			
			if (autoDrive(forward) < 150) {
				leftMaster.set(0);
				rightMaster.set(0);
				turretMotor.set(0);
				hoodMotor.set(0);
				gearArmMotor.set(0);
				resetPID();
		
				leftMaster.setEncPosition(0);
				rightMaster.setEncPosition(0);
				
				if(turnAngle != 0){
					
					gearShootAuton = GearAuton.turn;
				
				}
				else{
					
					gearShootAuton = GearAuton.drop;
				}
			}
			break;

		case turn:
			autoHood(hoodAngle);
			autoTurret(turretAngle);
			if ((absVal(horzGyro.getAngle() - turnAngle)) < 1) {
				leftMaster.set(0);
				rightMaster.set(0);
				gearShootAuton = GearAuton.drop;
			}
			break;

		case drop:
			autoHood(hoodAngle);
			autoTurret(turretAngle);
			
			if (autoGearArm(gearDrop) < 100 & autoDrive(reverse) < 650) {
				leftMaster.set(0);
				rightMaster.set(0);
				gearArmMotor.set(0);
				gearShootAuton = GearAuton.align;
			}
			break;

		case align:
			autoHood(hoodAngle);
			turretMotor.set(visionPID.GetOutput(table.getNumber("COG_X", 0), 0));
			autoShoot(wheelRPM);
			
			if(absVal(table.getNumber("COG_X", 100)) < 2){
				hoodMotor.set(0);
				turretMotor.set(0);
				gearShootAuton = GearAuton.shoot;
			}
			break;

		case shoot:
			autoHood(hoodAngle);
			autoShoot(wheelRPM);
			// turret.set(visionPID(165,table.getNumber("COG_X",0),visionKP));
			wheelOfDoomMotor.set(-0.45);
			lifterMotor.set(-1);
			kickerMotor.set(-1);
			
		}
		// wheel, lifterMotor, wheelgrabber, flywheel set to 0 in teleop

	}

	/*
	 * double visionPID(double target, double visionP){ double error = target -
	 * table.getNumber("COG_X", 216); double speed = error * visionP; return
	 * speed;
	 * 
	 * }
	 */

	void shootingAuto() {
		switch (shootingAuton) {
		// start revving up before turn
		case forward:
			//turretMotor.set(turretPID.GetOutput(turretMotor.getEncPosition(), turretAngle)); 
			leftMaster.set(driveLPID.GetOutput(leftMaster.getEncPosition(), distance));
			
			if (absVal(leftMaster.getEncPosition() - distance) < 200) {
				driveLPID.Reset();
			}
			
			rightMaster.set(driveRPID.GetOutput(rightMaster.getEncPosition(), -distance));
			
			if (absVal(absVal(rightMaster.getEncPosition()) - distance) < 200) {
				driveRPID.Reset();
			}
			
			if (absVal(getAverageEnc()) >= ((absVal(forward)) - 6)) {
				leftMaster.set(0);
				rightMaster.set(0);
				rightMaster.setEncPosition(0);
				rightSlave.setEncPosition(0);
				leftMaster.setEncPosition(0);
				leftSlave.setEncPosition(0);
				turretMotor.set(0);
				shootingAuton = BallAuton.turn;
			}
			break;
			
		case turn:
			leftMaster.set(normPID(-turnAngle, horzGyro.getAngle(), PTURN, ITURN));
			rightMaster.set(normPID(turnAngle, horzGyro.getAngle(), PTURN, ITURN));
			ballIntakeSolenoid.set(true);
			if (absVal(horzGyro.getAngle()) >= absVal(turnAngle) - 4) {
				leftMaster.set(0);
				rightMaster.set(0);
				rightMaster.setEncPosition(0);
				rightSlave.setEncPosition(0);
				leftMaster.setEncPosition(0);
				leftSlave.setEncPosition(0);
				shootingAuton = BallAuton.tacticalReload;
			}
			break;

		case tacticalReload:
			gearArmMotor.set(gearArmPID.GetOutput(gearArmMotor.getEncPosition(), 0));
			leftMaster.set(.8);
			rightMaster.set(-.8);
			//turretMotor.set(turretPID.GetOutput(turretMotor.getEncPosition(), turretAngle)); 
			
			
			if (turretMotor.getEncPosition() >= (turretAngle - 50)) {
				leftMaster.set(0);
				rightMaster.set(0);
				turretMotor.set(0);
				flyWheelMaster.changeControlMode(TalonControlMode.Speed);
				flyWheelMaster.set(autoRPM);// -4450
				shootingAuton = BallAuton.visionTrack;
			}
			break;

		case visionTrack:
			leftMaster.set(0.2);
			rightMaster.set(-0.2);
			if (absVal(getAverageEnc()) > 2000) {
				leftMaster.set(0);
				rightMaster.set(0);
			}
			kickerMotor.set(-1);
			lifterMotor.set(-1);
			climberMaster.set(0.65);
			
			// turrent.set(visionPID.GetOutput(table.getNumber("COG_X", 0.0),
			// 1)); //Parameters: ACTUAL, TARGET
			turretMotor.set(visionPID.GetOutput(table.getNumber("COG_X", 0.0), 1));
			if ((table.getNumber("COG_X", 0.0) > -3.0) && (table.getNumber("COG_X", 0.0) < 5.0)) {
				turretMotor.set(0);
				shootingAuton = BallAuton.stopAndShoot;
			}
			break;

		case stopAndShoot:
			leftMaster.set(0.2);
			rightMaster.set(-0.2);
			if (absVal(getAverageEnc()) > 2000) {
				leftMaster.set(0);
				rightMaster.set(0);
			}
			lifterMotor.set(-.7);
			kickerMotor.set(-.7);
			climberMaster.set(0.65);
			smartDash();
			if (flyWheelMaster.getSpeed() >= 3650.0) {
				wheelOfDoomMotor.set(-0.65);
				turretMotor.set(0.0);
				turretMotor.setEncPosition(0);
				hoodMotor.set(0.0);
			} else {
				wheelOfDoomMotor.set(0);
			}
			break;
		}
	}

	void resetPID(){
		driveLPID.Reset();
		driveRPID.Reset();
		hoodPID.Reset();
		gearArmPID.Reset();
		visionPID.Reset();	
	}
	
	double autoDrive(double target){
		driveSpeedLeft = driveLPID.GetOutput(leftMaster.getEncPosition(), -target);
		driveSpeedRight = driveRPID.GetOutput(rightMaster.getEncPosition(), target);
		turnSpeed = gyroPID.GetOutput(horzGyro.getAngle(), 0);
		
		double leftError = -target - leftMaster.getEncPosition();
		double rightError = target - rightMaster.getEncPosition();
		double avgError = (-leftError + rightError)/2;
		
		frontWheelSolenoid.set(true);
		backWheelSolenoid.set(false);
		
		
		leftMaster.set(driveSpeedLeft - turnSpeed);
		
		if (absVal(leftMaster.getEncPosition() - target) < 200) {
			driveLPID.Reset();
		}
		
		rightMaster.set(driveSpeedRight - turnSpeed);
		
		if (absVal(absVal(rightMaster.getEncPosition()) - target) < 200) {
			driveRPID.Reset();
		}
		
		if(absVal(horzGyro.getAngle()) < .5){
			gyroPID.Reset();
		}

		return absVal(avgError);
	}
	
	double autoHood(double target){
		double hoodError = target - hoodEncoder.getValue();
		
		hoodMotor.set(-hoodPID.GetOutput(hoodEncoder.getValue(), target));
		
		return absVal(hoodError);
	}
	
	double autoTurret(double target){
		double turretError = turretMotor.getEncPosition() - target;
		if(turretError > 100){
			turretMotor.set(-.35);
		}
		
		else if(turretError < -100){
			turretMotor.set(.35);
		}
		
		else{
			turretMotor.set(0);
		}

		return absVal(turretError);
	}
	
	double autoGearArm(double target){
		double gearError = target - gearArmMotor.getEncPosition(); 
		
		gearArmMotor.set(gearArmPID.GetOutput(gearArmMotor.getEncPosition(), target));
		
		return absVal(gearError);
	}
	

	
	double autoShoot(double target){
		flyPID = flyWheelPID.GetOutput(flyWheelMaster.getSpeed(), wheelRPM);
		double shootError = target - flyPID;
		
		flyWheelMaster.set(flyWheelPID.GetOutput(flyWheelMaster.getSpeed(), target));
		
		return shootError;
	}
	
	double getAverageEnc() {
		double avgEnc = ((absVal(leftMaster.getEncPosition()) + absVal((rightMaster.getEncPosition()))) / 2);
		return avgEnc;
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void teleopInit() {
		flyWheelMaster.setEncPosition(0);
		rightMaster.setEncPosition(0);
		rightSlave.setEncPosition(0);

		leftMaster.setEncPosition(0);
		leftSlave.setEncPosition(0);
		leftMaster.setCurrentLimit(40);
		rightMaster.setCurrentLimit(40);
		leftSlave.setCurrentLimit(40);
		rightSlave.setCurrentLimit(40);
		
		wheelRPM = 3620;
		flyWheelPID.SetP(0.0025);
		flyWheelPID.SetI(1.0E-9);
		flyWheelPID.SetD(0.0072);
	}

	@Override
	public void teleopPeriodic() {

		if (turretMotor.getEncPosition() <= -4125) {
			turretFlip = false;
		}

		// System.out.println("rightMaster: " + rightMaster.getEncPosition() +
		// "leftMaster: " + leftMaster.getEncPosition() + "rightSlave: " +
		// rightSlave.getEncPosition() + "leftSlave: " +
		// leftSlave.getEncPosition());
		if (driver.getRawButton(leftTrigger)) {
			climberMaster.set(1);
		} else {
			climberMaster.set(0);
		}

		gearManipulator();
		shooter();
		hood();
		climber();
		drivebase();
		smartDash();
		// System.out.println("Front Left: " + leftMaster.getEncPosition() + "
		// Front Right: " + rightMaster.getEncPosition() + " Back Left: " +
		// leftSlave.getEncPosition() + " Back Right: " +
		// rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());
		// System.out.println("P: " + flyP + " I: " + flyI + " D: " + flyD+ " F:
		// " +flyWheel.getF());
	}

	void hood() {
		// System.out.println("Hood Position: " +hoodEnc.getValue());
		// hood.set(-hoodPID(770,hoodEnc.getValue()));
		// if(oper.getPOV() == 0){
			// flyWheelSpeed = -3550;
		 //}
		 //if(oper.getPOV() == 90){
			// flyWheelSpeed = -3600;
		 //}
		// if(oper.getPOV() == 180){
		// flyWheelRPM = -3650;
		// }
		// if(oper.getPOV() == 270){
		// flyWheelRPM = -3700;
		// }

		if (oper.getRawButton(rightBumper)) {
			hoodMotor.set(.5);
		} else {
			if (oper.getRawButton(rightTrigger)) {
				hoodMotor.set(-.5);
			} else {
				hoodMotor.set(0);
			}
		}
		
		if(driver.getRawButton(buttonCircle)){
			hoodMotor.set(-hoodPID.GetOutput(hoodEncoder.getValue(), hoodAngle));
		}

	}

	void drivebase() {
		double speed = deadBand(-driver.getY());
		double turn = deadBand(-driver.getZ());

		rightMaster.set((speed + turn));
		if(absVal(speed) > 0.5){
			speed *= .91;
		} 
		leftMaster.set((-speed + turn));

		
		// Solenoids
		if (driver.getRawButton(leftBumper)) {
			frontWheelSolenoid.set(true); // omni
			backWheelSolenoid.set(true); // omni
		}

		if (driver.getRawButton(rightBumper)) {
			frontWheelSolenoid.set(false);// traction
			backWheelSolenoid.set(false);// traction
		}
		
		if(driver.getRawButton(rightTrigger)) { //half and half
			frontWheelSolenoid.set(true);
			backWheelSolenoid.set(false);
		}
	

		if (driver.getRawButton(1)) {
			rightMaster.setEncPosition(0);
			rightSlave.setEncPosition(0);
			leftMaster.setEncPosition(0);
			leftSlave.setEncPosition(0);
		}
	}
	
	void climber() {
		if(driver.getRawButton(leftTrigger)) {
			climberMaster.set(1);
		}
		
		else {
			climberMaster.set(0);
		}
	}

	double deadBand(double joyStick) {
		if (joyStick > -0.1 && joyStick < 0.1) {
			joyStick = 0;
		}
		return joyStick;
	}

	void gearManipulator(){ 
		//200 is carry 
		//800 is drop 
			if(oper.getRawAxis(1) > .2){ //up position for gear arm 1264
				gearArmMotor.set(0.30);
			}
			else{
				if(oper.getRawAxis(1) < -.2){ //deliver position for gear arm
					gearArmMotor.set(-0.30);
				}
				else{
					if(oper.getRawButton(buttonSquare)){
						gearArmMotor.set(gearArmPID.GetOutput(gearArmMotor.getEncPosition(), gearCarry));
					}
					else if(oper.getRawButton(buttonX)){
						gearArmMotor.set(gearArmPID.GetOutput(gearArmMotor.getEncPosition(), gearPlace));
					}
					else{
						gearArmMotor.set(0);
						gearArmPID.Reset();
					}
				}
			}
			if(oper.getRawButton(leftBumper)){
				gearIntakeMotor.set(-.75);
			}
			else{
				if(oper.getRawButton(leftTrigger)){
					gearIntakeMotor.set(.75);
				}
				else{
					gearIntakeMotor.set(0);
				}
			}
			if(oper.getRawButton(buttonPS)){
				gearArmMotor.setEncPosition(0);
				turretMotor.setEncPosition(0);
			}
			
			
		
	}
	
	
	

	/*
	 * void shooterTest(){ flywheelPID.SetPID(flyKp, flyKi, flyKd);
	 * motor.set(flywheelPID.GetOutput(motor.getEncVelocity(), flywheelRPM)); }
	 */

	void ballSystem() {
		runIntake();
	}

	void runIntake() {
		ballIntakeMotor.set(deadBand(oper.getRawAxis(rightAnalog)));
	}

	
	void shooter() {
		if(absVal(table.getNumber("COG_X", 0)) < 2){
			visionPID.Reset();
		}
		if (driver.getPOV() == 90) {
			turretMotor.set(-.35);
		} else {
			if (driver.getPOV() == 270) {
				turretMotor.set(0.35);
			} else {
				if (driver.getRawButton(buttonSquare)) {
					turretMotor.set(vPID);

				} else {
					turretMotor.set(0);
					visionPID.Reset();
				}
			}
		}
		if(oper.getPOV() == 0) {
			wheelRPM = 3620;
		}
		if(oper.getPOV() == 90) {
			wheelRPM = 3640;
		}
		if(oper.getPOV() == 180) {
			wheelRPM = 3660;
		}
		if(oper.getPOV() == 270) {
			wheelRPM = 3680;
		}
		
		ballIntakeMotor.set(-(deadBand(oper.getRawAxis(leftBumper)))); //Axis set to a bumper?
		// shooting

		if (oper.getRawButton(buttonTriangle)) {
			flyWheelMaster.set(flyWheelSpeed);
			flyWheelMaster.set(-flyWheelPID.GetOutput(flyWheelMaster.getSpeed(), flyWheelSpeed));
			lifterMotor.set(-1);
			kickerMotor.set(-1);

		} else {
			flyWheelMaster.changeControlMode(TalonControlMode.PercentVbus);
			flyWheelMaster.set(0);
			lifterMotor.set(0);
			kickerMotor.set(0);
		}
		if (oper.getRawButton(buttonCircle)) {
			wheelOfDoomMotor.set(-0.75);
		} else {
			if (oper.getRawButton(rightAnalog)) {
				wheelOfDoomMotor.set(1);
			} else {
				wheelOfDoomMotor.set(0);
			}
		}
	}

	double normPID(double target, double position, double pVal, double iVal) {
		double PIDerror = target - position;
		cumulativePIDError += PIDerror;
		double PIDPout = PIDerror * pVal;
		double PIDIout = cumulativePIDError * iVal;
		double PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDerror) < 20) {
			PIDSpeed = 0;
		}

		return PIDSpeed;
	}

	double absVal(double input) {
		if (input < 0) {
			return -input;
		} else {
			return input;
		}
	}
	
	public void disabledPeriodic(){
		if(resetSwitch.get() == false){
			horzGyro.calibrate();
			leftMaster.setEncPosition(0);
			rightMaster.setEncPosition(0);
			turretMotor.setEncPosition(0);
			gearArmMotor.setEncPosition(0);
		}
	}

	void smartDash() {

		SmartDashboard.putNumber("Speed", flyWheelMaster.getSpeed());
		SmartDashboard.putNumber("COG_X", table.getNumber("COG_X", 0));
		// SmartDashboard.putNumber("COG_Y", table.getNumber("COG_Y",0));
		 SmartDashboard.putNumber("Gear Arm Encoder", gearArmMotor.getEncPosition());
		// gearArmMotor.getEncPosition());
		 SmartDashboard.putNumber("Hood Encoder", hoodEncoder.getValue());
		 SmartDashboard.putNumber("Gyro Angle", horzGyro.getAngle());
		 SmartDashboard.putNumber("Front Left Encoder", leftMaster.getEncPosition());
		// SmartDashboard.putNumber("Back Left Encoder",
		// leftSlave.getEncPosition());
		 SmartDashboard.putNumber("Front Right Encoder", rightMaster.getEncPosition());
		 SmartDashboard.putNumber("RightM current", rightMaster.getOutputCurrent());
		 SmartDashboard.putNumber("leftM current", leftMaster.getOutputCurrent());
		 SmartDashboard.putNumber("RightS current", rightSlave.getOutputCurrent());
		 SmartDashboard.putNumber("leftS current", leftSlave.getOutputCurrent());
		 SmartDashboard.putNumber("turn speed", turnSpeed);
		// SmartDashboard.putNumber("Back Right Encoder",
		// rightSlave.getEncPosition());
		SmartDashboard.putNumber("Turret Encoder", turretMotor.getEncPosition());
		// SmartDashboard.putNumber("Fly Wheel Master Current",
		// flyWheelMaster.getOutputCurrent());
		// SmartDashboard.putNumber("Fly Wheel Slave Current",
		// flyWheelSlave.getOutputCurrent());
		// SmartDashboard.putNumber("wod Current",
		// wheelOfDoomMotor.getOutputCurrent())
		
		//wheelRPM = pref.getDouble("RPM", 3620);
		//hoodAngle = pref.getDouble("Hood", 770);
		//driveP = pref.getDouble("driveP", 0);
		//driveI = pref.getDouble("driveI", 0);
		//driveD = pref.getDouble("driveD", 0);
//		distance = pref.getDouble("Distance", 0);
//		gyroP = pref.getDouble("gyroP", 0);
//		gyroI = pref.getDouble("gyroI", 0);
//		gyroD = pref.getDouble("gyroD", 0);
//		turretP = pref.getDouble("turretP", 0);
//		turretI = pref.getDouble("turretI", 0);
//		turretD = pref.getDouble("turretD", 0);
		//hoodP = pref.getDouble("hoodP", 0);
		//hoodI = pref.getDouble("hoodI", 0);
		//hoodD = pref.getDouble("hoodD", 0);
		//turnSpeed = pref.getDouble("turnSpeed", 0);
		//turnValue = pref.getDouble("turnValue", 0);
		
//		gyroPID.SetP(gyroP);
//		gyroPID.SetI(gyroI); 
//		gyroPID.SetD(gyroD); 
		
//		driveRPID.SetP(driveP);
//		driveRPID.SetI(driveI); 
//		driveRPID.SetD(driveD); 
//		driveLPID.SetP(driveP);
//		driveLPID.SetI(driveI); 
//		driveLPID.SetD(driveD); 
		
		//hoodPID.SetP(hoodP);
		//hoodPID.SetI(hoodI);
		//hoodPID.SetD(hoodD);
		
		//turretPID.SetP(turretP);
		//turretPID.SetI(turretI);
		//turretPID.SetD(turretD);
		

	}
}
