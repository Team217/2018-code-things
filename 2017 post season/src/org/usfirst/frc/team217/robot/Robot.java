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




/**x
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

//Hood angle is 745

public class Robot extends IterativeRobot {
	final String redSide = "Red Auto";
	final String blueSide = "Blue Auto";
	final String ballAuton = "40 Ball Auto";
	final String gearAuton = "Gear and Shoot Auto";
	
	final String left = "Left";
	final String center = "Center";
	final String right = "Right";
	
	String sideSelected, autoSelected, positionSelected;
	//boolean turretFlip=true; 
	
	PIDController gyroPID;
	
	double flywheelPID;
	
	boolean turretFlip=true;
	
	//Joysticks
	Joystick oper, driver;
	Preferences prefs;
	
	//Joystick Variables
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
	
	
	//PID flyWheelPID;
	double wheelRPM, hoodAngle, flyKp, flyKi, flyKd, flyWheelSpeed, wheelOfDoomSpeed; 
	double flyP, flyI, flyD, flyF;
	double visionP, visionI, visionD;
	
	//Drive Motors
	//left motor is flipped
	CANTalon rightMaster, rightSlave, leftMaster, leftSlave;
	
	//Shooting Motors
	CANTalon flyWheelMaster, flyWheelSlave, hoodMotor, turretMotor, lifterMotor, kickerMotor, ballIntakeMotor, wheelOfDoomMotor;
	
	//Climbing Motors
	CANTalon climberMaster, climberSlave;
	
	//Gear Motors
	CANTalon gearArmMotor, gearIntakeMotor;
	
	//Solenoids
	Solenoid backWheelSolenoid, frontWheelSolenoid, ballIntakeSolenoid;
	
	//Gyro
	ADXRS450_Gyro horzGyro;
	
	DigitalInput resetSwitch;
	
	//Compressor
	Compressor compressor;
	
	Preferences pref;
	
	AnalogInput hoodEncoder;
	
	NetworkTable table, visionTable;
	
	PID flyWheelPID = new PID(0,0,0);
	PID visionPID = new PID(0.0217,0,0);
	
	boolean camNum = false, autonEncReset = true;
//	double flyWheelRPM = -4400;
	
	enum BallAuton{
		forward,
		turn,
		tacticalReload,
		visionTrack,
		stopAndShoot
	};
	BallAuton shootingAuton;
	
	enum GearAuton{
		forward,//same for each side of the field, different for positions
		turn, //same for each side of the field, different for positions
		pivot,//same for everything -- for gear arm
		place,//same
		drop,//same 
		align,//different for all
		shoot,//different for positions
		dontMove//same for all
	}
	GearAuton gearShootAuton;
	
	enum DriveForward{
		drive,
		stop
	}
	DriveForward driveAuto;
	Timer gearTime; //Placed here for reference, encoder will be used instead
	
	//shootingAuto variables
		//turn for blue = -90
		//turn for red = 87
		//turret angle for blue = -2590 
		//turret angle for red = -2530
		
		double climbSpeed; 
		int autoRPM;//change based on field, might have to be different for each side 
		int forward1;//change based on field, might have to be different for each side
		
	//gearAuto variables 
		//turn for blue = 82
		//turn for red = 
		 
		int forward; 
	
	//will be used in both shootingAuton and gearAuton
	int turretAngle; //change based on field
	double turnAngle;//change based on field
	
	//PID variables
	double cumulativePIDError = 0;
	//PID P val for going straight
	final double PSTRAIGHT = .00217;
	//PID P val for turning
	final double PTURN = .02;
	//PID I value for turning
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
		
		//gearTime = new Timer(); 
		
		//Smart Dashboard
		side.addDefault("Red Alliance", redSide);
		side.addObject("Blue Alliance", blueSide);
		SmartDashboard.putData("Auton Side Selection", side);

		auton.addDefault("40 Ball", ballAuton);
		auton.addObject("10 Ball Gear Auto", gearAuton);
		SmartDashboard.putData("Auton Selection",auton);

		position.addDefault("Left", left);
		position.addObject("Center", center);
		position.addObject("Right", right);
		SmartDashboard.putData("Position Selection",position);


		pref = Preferences.getInstance();

		hoodEncoder = new AnalogInput(1);
		
		
		//Gear Manipulator
		gearArmMotor = new CANTalon(8);
		gearArmMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		gearIntakeMotor = new CANTalon(7);
		
		//Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);
		
		//resetSwitch = new DigitalInput(1);
		
		//Drive Motors
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
		
		//Shooting Motors
		flyWheelMaster = new CANTalon(10);
		flyWheelMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		flyWheelMaster.reverseSensor(false);
		flyWheelSlave = new CANTalon(11);
		flyWheelSlave.changeControlMode(TalonControlMode.Follower);
		flyWheelSlave.set(10);
		
		hoodMotor = new CANTalon(9);
		turretMotor= new CANTalon(6);
		turretMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		lifterMotor = new CANTalon(5);
		kickerMotor = new CANTalon(4);
		ballIntakeMotor =  new CANTalon(13);
		wheelOfDoomMotor = new CANTalon(12);
		
		//flyWheel Settings
		flyWheelMaster.configNominalOutputVoltage(+0.0f,  -0.0f);
		flyWheelMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		
		
		//Climbing Motors
		climberMaster = new CANTalon(2);
		climberSlave = new CANTalon(3);
		climberSlave.changeControlMode(TalonControlMode.Follower);
		climberSlave.set(2);
		
		
		//Solenoids
		frontWheelSolenoid = new Solenoid(2);
		backWheelSolenoid = new Solenoid(1);
		ballIntakeSolenoid = new Solenoid(3);
		
		//Gyro
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
	
	}
@Override
	public void autonomousInit() {
		encReset();
		sideSelected = (String) side.getSelected();
		autoSelected = (String) auton.getSelected();
		positionSelected = (String) position.getSelected();
		
		switch(autoSelected){
			case gearAuton: 
				gearAutoInit(sideSelected.equals(blueSide));
				break;
			case ballAuton:
				//call in function 
				//pass in boolean param 
				//blue=true
				shootingAutoInit(sideSelected.equals(blueSide));
				break;
		}
		
		
	}
	
	void encReset(){
		rightMaster.setEncPosition(0);
		rightSlave.setEncPosition(0);
		leftMaster.setEncPosition(0);
		leftSlave.setEncPosition(0);
		turretMotor.setEncPosition(0);
		ballIntakeMotor.setEncPosition(0);
		horzGyro.reset();
	}

	//when boolean blue = true, the bot is on the blue side
	//when boolean blue = false, bot is on red side
	void shootingAutoInit(boolean blue){

		if (blue){
			turnAngle = -90;
			turretAngle = -2590;
		}
		
		else{ 
			turnAngle = 87;
			turretAngle = -2530;
			climbSpeed = .65;
		}
		
		autoRPM = -4450; // Change RPM through testing, increase value for both high and long shots, use hood vale 
		forward1 = 4400; 
		turretMotor.setEncPosition(0);
		frontWheelSolenoid.set(true);
		backWheelSolenoid.set(false);
		shootingAuton = BallAuton.forward;
	}
	
	void gearAutoInit(boolean blue){
		boolean shoot; 
		
		horzGyro.reset();
		leftMaster.setEncPosition(0);
		rightMaster.setEncPosition(0);
		gearArmMotor.setEncPosition(0);
		frontWheelSolenoid.set(false);
		backWheelSolenoid.set(false);
		gearShootAuton = GearAuton.forward;
		
		switch(positionSelected){
			case left:
				turnAngle = 82;
				if(blue){
					shoot = true;
					forward = 6500;
				}
				else{
					shoot = false;
					forward = 7570;
					
				}
				break;
				
			case center:
				turnAngle = 0;
				forward = 3440;
				if(blue){
					shoot = true;
					turretAngle = 2200;
					
				}
				else{
					shoot = true;
					turretAngle = 8000;
					
				}
				break;
			
			case right: 
				turnAngle = -82;
				if(blue){
					shoot = false;
					forward = 5650;
					turretAngle = 2500;
						
				}
				else{
					shoot = true;
					
				}
				break;
		
			
					
		}	
		
	}



/*
					Gear stuff, pos 1:
							Different: Turn- Blue: 82 Red: -56
									   Forward: Left: Blue: -6400 Red: -7470
									   			Right: Blue: 6600 Red: 7670
									   	Timers: Red has timer? Whaaaaa
							Same: Resetting enc positions
						
					Gear stuff, pos 2:						
									   	Turret: Blue: 2200 Red: 8000
							Same: Solenoid both false
								  Enc Position reset
								  Forward: Left: Blue: -3370 Red: -3370
								  Right: Blue: 3470 Red: 3470
							
*/


	/**
	 * This function is called periodically during autonomous
	 */
  
	@Override
	public void autonomousPeriodic() {
		if(autoSelected.equals(gearAuton)){
			gearAuto();
		}
		if(autoSelected.equals(ballAuton)){
			shootingAuto();
		}
	}
	
	void gearAuto(){
		switch(gearShootAuton){
			case forward:
				leftMaster.set(normPID(-forward, getAverageEnc(), PSTRAIGHT, 0));
				rightMaster.set(normPID(forward, getAverageEnc(), PSTRAIGHT, 0));
				
				smartDash();
				
				if(getAverageEnc() > forward){
					leftMaster.set(0);
					rightMaster.set(0);
					gearShootAuton = GearAuton.turn;
				}
				break;
			
			case turn:
				if(positionSelected.equals(center)){
					leftMaster.set(0);
					rightMaster.set(0);
					gearShootAuton = GearAuton.pivot;
				}
			
				leftMaster.set(gyroPID(turnAngle, PTURN));
				rightMaster.set(gyroPID(turnAngle, PTURN));
			
				if((absVal(horzGyro.getAngle() - turnAngle)) < 1){
					leftMaster.set(0);
					rightMaster.set(0);
					gearShootAuton = GearAuton.pivot;
				}
			break;
			
			case pivot:
				gearArmMotor.set(.5);
				if(gearArmMotor.getEncPosition() >= 310){
					gearArmMotor.set(0);
					leftMaster.setEncPosition(0);
					rightMaster.setEncPosition(0);
					gearShootAuton = GearAuton.place;
				}
			break;
			
			case place:
				leftMaster.set(normPID(-forward, getAverageEnc(), PSTRAIGHT, 0));
				rightMaster.set(normPID(forward, getAverageEnc(), PSTRAIGHT, 0));
				gearArmMotor.set(normPID(330,gearArmMotor.getEncPosition(),0.00097,0));

				smartDash();
				
				/*if(gearTime.get()>=gearAutoTime){
					leftMaster.set(0);
					rightMaster.set(0);
					gearBallAuto = GearAuto.drop;
				}*/
			break;
			
			case drop:
				gearArmMotor.set(normPID(1100, gearArmMotor.getEncPosition(), 0.00097, 0));
				/*if(1080 <= gearArmMotor.getEncPosition() <= 1120){
					leftMaster.set(-.35);
					rightMaster.set(.4);
				}*/
			
				turretMotor.set(normPID(-turretAngle, turretMotor.getEncPosition(), 0.0011, 0));
				flyWheelMaster.changeControlMode(TalonControlMode.Speed);
				flyWheelMaster.set(-4850);
				//lifter.set(-1);
				//kicker.set(-1);
				//hood.set(-hoodPID(222,hoodEnc.getValue()));
				//leftMaster.getEncPosition() >= 6000 && rightMaster.getEncPosition() <= -6100
				/*if(gearTime.get() >=(gearAutoTime+1.7)){
					frontWheelSolenoid.set(true);//This deploys omni if not already deployed.
					backWheelSolenoid.set(true);
					leftMaster.set(0);
					rightMaster.set(0);
					gearArmMotor.set(0);
					gearShootAuton = GearAuton.align;
			}*/
			break;
			
			case align:
				/*turretMotor.set(visionPID(0, .00217));
				(absVal(table.getNumber() < 2)){
					turretMotor.set(0);
					gearShootAuton = GearAuton.align;
				}*/
				break;
				
			case shoot:
				//turret.set(visionPID(165,table.getNumber("COG_X",0),visionKP));
				wheelOfDoomMotor.set(-1);
				lifterMotor.set(-.7);
				kickerMotor.set(-.7);
			
		}
		//wheel, lifterMotor, wheelgrabber, flywheel set to 0 in teleop

	}
	
	/*double visionPID(double target, double visionP){
		double error = target - table.getNumber("COG_X", 216);
		double speed = error * visionP;
		return speed;

	}*/
	
	
	void shootingAuto(){
		switch(shootingAuton){
		//start revving up before turn
			case forward:
				turretMotor.set(normPID(turretAngle, turretMotor.getEncPosition(), 0.0011, 0));
				leftMaster.set(normPID(-forward, leftMaster.getEncPosition(), PSTRAIGHT, 0));
				rightMaster.set(normPID(forward, rightMaster.getEncPosition(), PSTRAIGHT, 0));
				if(absVal(getAverageEnc()) >= ((absVal(forward)) - 6)){
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
				if(absVal(horzGyro.getAngle()) >= absVal(turnAngle) -4){
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
				ballIntakeMotor.set(normPID(0, ballIntakeMotor.getEncPosition(), 0.00049, 0)); //Real dumb pid values, what the heck is gweely doing
				leftMaster.set(.8);
				rightMaster.set(-.8);
				turretMotor.set(normPID(turretAngle, turretMotor.getEncPosition(), 0.0011, 0)); //More dumb pid what
				if(turretMotor.getEncPosition() >= (turretAngle-50)){
					leftMaster.set(0);
					rightMaster.set(0);
					turretMotor.set(0);
					flyWheelMaster.changeControlMode(TalonControlMode.Speed);
					flyWheelMaster.set(autoRPM);//-4450
					shootingAuton = BallAuton.visionTrack;
				}
			break;
			
			case visionTrack:
				leftMaster.set(0.2);
				rightMaster.set(-0.2);
			if(absVal(getAverageEnc()) > 2000){
				leftMaster.set(0);
				rightMaster.set(0);
			}	
			kickerMotor.set(-1);
			lifterMotor.set(-1);
			climberMaster.set(0.65);
			smartDash();
			//turrent.set(visionPID.GetOutput(table.getNumber("COG_X", 0.0), 1)); //Parameters: ACTUAL, TARGET
			turretMotor.set(visionPID.GetOutput(table.getNumber("COG_X", 0.0), 1));
			if ((table.getNumber("COG_X", 0.0) > -3.0) && (table.getNumber("COG_X", 0.0) < 5.0)) {
				turretMotor.set(0);
				shootingAuton = BallAuton.stopAndShoot;
			}
			break;
			
			case stopAndShoot: 
			leftMaster.set(0.2);
			rightMaster.set(-0.2);
			if(absVal(getAverageEnc()) > 2000){
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
    		 }
    		else{
    			wheelOfDoomMotor.set(0);
    		}
    		break;
		}
	}

		
	double getAverageEnc(){
		double avgEnc = ((absVal(leftMaster.getEncPosition()) + absVal((rightMaster.getEncPosition()))) / 2);
		return avgEnc;
	}
	/**
	 * This function is called periodically during operator control
	 */
	
	public void teleopInit(){
		flyWheelMaster.setEncPosition(0);
		rightMaster.setEncPosition(0);
		rightSlave.setEncPosition(0);
		
		leftMaster.setEncPosition(0);
		leftSlave.setEncPosition(0);	
		leftMaster.setCurrentLimit(40);
		rightMaster.setCurrentLimit(40);
		leftSlave.setCurrentLimit(40);
		rightSlave.setCurrentLimit(40);
	}
	@Override
	public void teleopPeriodic() {
		
		if(turretMotor.getEncPosition()<= -4125){
			turretFlip = false;
		}
		
		//System.out.println("rightMaster:  " + rightMaster.getEncPosition() + "leftMaster:  " + leftMaster.getEncPosition() + "rightSlave:  " + rightSlave.getEncPosition() + "leftSlave:  " + leftSlave.getEncPosition());
		if(driver.getRawButton(leftTrigger)){
			climberMaster.set(1);
		}else{
			climberMaster.set(0);
		}
		
		//gearManipulator();
		visionTracking();
		shooter();
		hood();
		drivebase();
		smartDash();
		//System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());
		//System.out.println("P: " + flyP + " I: " + flyI + " D: " + flyD+ " F: " +flyWheel.getF());
	}
	
	void visionTracking(){
		
		if(oper.getRawButton(buttonSquare)){
			turretMotor.changeControlMode(TalonControlMode.PercentVbus);
			turretMotor.set(visionPID.GetOutput(table.getNumber("COG_X", 0), 0));
		}
		else {
			turretMotor.changeControlMode(TalonControlMode.PercentVbus);
		}
	}
	void hood(){
		//System.out.println("Hood Position: " +hoodEnc.getValue());
		//hood.set(-hoodPID(770,hoodEnc.getValue()));
//		if(oper.getPOV() == 0){
//			flyWheelRPM = -3550;
//		}
//		if(oper.getPOV() == 90){
//			flyWheelRPM = -3600;
//		}
//		if(oper.getPOV() == 180){
//			flyWheelRPM = -3650;
//		}
//		if(oper.getPOV() == 270){
//			flyWheelRPM = -3700;
//		}
		
		if(oper.getRawButton(rightBumper)){
			hoodMotor.set(.5);
		}
		else{
			if(oper.getRawButton(rightTrigger)){
				hoodMotor.set(-.5);
			}
			else{
				hoodMotor.set(0);
			}
		}

	}

	void drivebase(){
		double speed = deadBand(-driver.getY());
		double turn = deadBand(-driver.getZ());
		
		leftMaster.set((-speed + turn));
		rightMaster.set((speed + turn));
				
		//Solenoids
		if(driver.getRawButton(leftBumper)){
			frontWheelSolenoid.set(true); //omni
			backWheelSolenoid.set(true);  //omni
		}
		
		if(driver.getRawButton(rightBumper)){
			frontWheelSolenoid.set(false);//traction
			backWheelSolenoid.set(false);//traction
		}
		
		if(driver.getRawButton(1)){
			rightMaster.setEncPosition(0);
			rightSlave.setEncPosition(0);
			leftMaster.setEncPosition(0);
			leftSlave.setEncPosition(0);
		}
	}
	
	double deadBand(double joyStick){
		if(joyStick > -0.1 && joyStick < 0.1){
			joyStick = 0;
		}
		return joyStick;
	}
	
	void gearManipulator(){
		gearArmMotor.set(deadBand(oper.getRawAxis(leftAnalog)));
		if(oper.getRawButton(leftTrigger)){
			gearIntakeMotor.set(1);
		}
		else if(oper.getRawButton(rightTrigger)){
			gearIntakeMotor.set(-1);
		}
		else{
			gearIntakeMotor.set(0);
		}
	} 
	
	/*void shooterTest(){
		flywheelPID.SetPID(flyKp, flyKi, flyKd);
		motor.set(flywheelPID.GetOutput(motor.getEncVelocity(), flywheelRPM));
	}*/
	
	void ballSystem(){
		runIntake();
	}
	
	void runIntake(){
		ballIntakeMotor.set(deadBand(oper.getRawAxis(rightAnalog)));
	}
	//wod neg
	void shooter(){
				if(oper.getRawButton(buttonShare)){
					turretMotor.set(-.5);
				}
				else{
					if(oper.getRawButton(buttonOption)) {
						turretMotor.set(0.5);
					}
					else {
					turretMotor.set(0);
				}
				}
			ballIntakeMotor.set(-(deadBand(oper.getRawAxis(leftBumper))));
			//shooting
		
			if(oper.getRawButton(buttonTriangle)){
				//flyWheelMaster.changeControlMode(TalonControlMode.Speed);
				flyWheelMaster.set(flyWheelSpeed);
				//flyWheelMaster.set((-normPID(flyWheelRPM, flyWheelMaster.getEncVelocity(), flyKp, flyKi)));
				flyWheelMaster.set(-flyWheelPID.GetOutput(flyWheelMaster.getSpeed(), flyWheelSpeed));
				System.out.println(flyWheelMaster.getSpeed());
				lifterMotor.set(-1);
				kickerMotor.set(-1);
				
			}else{
				flyWheelMaster.changeControlMode(TalonControlMode.PercentVbus);
				flyWheelMaster.set(0);
				lifterMotor.set(0);
				kickerMotor.set(0);
			}
			if(oper.getRawButton(buttonCircle)){
				wheelOfDoomMotor.set(wheelOfDoomSpeed);													
			}else{
				if(oper.getRawButton(rightAnalog)){
					wheelOfDoomMotor.set(1);
				}else{
					wheelOfDoomMotor.set(0);
				}
			}
	}
	
	double normPID(double target, double position, double pVal, double iVal){
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
	
	double absVal(double input){
		if ( input < 0){
			return -input;
		}
		else{
			return input;
		}
	}
	
	double gyroPID(double target, double pVal){
		double gyroError = target - horzGyro.getAngle();
		
		if(absVal(gyroError) < 2){
			return 0;
		}
		else{
			return gyroError * pVal;
		}
	}
	
	
	void smartDash(){

		SmartDashboard.putNumber("Speed", flyWheelMaster.getSpeed());
		SmartDashboard.putNumber("COG_X", table.getNumber("COG_X",0));
		//SmartDashboard.putNumber("COG_Y", table.getNumber("COG_Y",0));
		//SmartDashboard.putNumber("Gear Arm Encoder", gearArmMotor.getEncPosition());
		SmartDashboard.putNumber("Hood Encoder", hoodEncoder.getValue());
		//SmartDashboard.putNumber("Gyro Angle", horzGyro.getAngle());
		//SmartDashboard.putNumber("Front Left Encoder", leftMaster.getEncPosition());
		//SmartDashboard.putNumber("Back Left Encoder", leftSlave.getEncPosition());
		//SmartDashboard.putNumber("Front Right Encoder", rightMaster.getEncPosition());
		//SmartDashboard.putNumber("Back Right Encoder", rightSlave.getEncPosition());
		//SmartDashboard.putNumber("Turret Spin Encoder", turretMotor.getEncPosition());
		//SmartDashboard.putNumber("Fly Wheel Master Current", flyWheelMaster.getOutputCurrent());
		//SmartDashboard.putNumber("Fly Wheel Slave Current", flyWheelSlave.getOutputCurrent());
		//SmartDashboard.putNumber("wod Current", wheelOfDoomMotor.getOutputCurrent());


		//System.out.println(table.getNumber("COG_X",216));
		//System.out.println(table.getNumber("COG_Y",216));

		flyP = pref.getDouble("P",0);
		flyI = pref.getDouble("I",0);
		flyD = pref.getDouble("D",0);
		//flyF = pref.getDouble("F",0);
		
		visionP = pref.getDouble("visionP", 0);
		visionI = pref.getDouble("visionI", 0);
		visionD = pref.getDouble("visionD", 0);

		wheelOfDoomSpeed = pref.getDouble("WheelOfDoom", 1);
		flyWheelSpeed = pref.getDouble("FlyWheelSpeed", 1);

		//intakeSpeed = pref.getDouble("Intake", 1);
		//robotSpeed = pref.getDouble("Speed",1);
		wheelRPM = pref.getDouble("RPM", 3240);
		hoodAngle = pref.getDouble("Hood", 745);
		//visionKP = pref.getDouble("VisionP", 0.00217);
		//autoP = pref.getDouble("AutonGyroP", 0.0099);
		//visionKI = pref.getDouble("VisionI", 0.00217);

		//autoForwardLeft = pref.getDouble("ForwardLeft", 27);
		//autoForwardRight = pref.getDouble("ForwardRight", 27);


		flyWheelPID.SetP(flyP); //0.1
		flyWheelPID.SetI(flyI); //0.0004895
		flyWheelPID.SetD(flyD); //0.5
		
		visionPID.SetP(visionP);
		visionPID.SetI(visionI);
		visionPID.SetD(visionD);
		
	}
}
