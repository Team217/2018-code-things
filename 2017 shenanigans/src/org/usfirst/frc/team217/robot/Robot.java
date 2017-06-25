package org.usfirst.frc.team217.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import java.lang.Math;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String redSide = "Red Auto";
	final String blueSide = "Blue Auto";
	final String ballsOut = "40 Ball Auto";
	final String gearShoot = "Gear and Shoot Auto";
	final String pos1 = "Position 1";
	final String pos2 = "Position 2";
	final String pos3 = "Position 3";
	
	String sideSelected, autoSelected, positionSelected;
	
	PIDController gyroPID;
	
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
	
	
	double wheelRPM, hoodValue, flyP,flyI,flyD,flyF,autoForwardLeft,autoForwardRight;
	
	//Drive Motors
	CANTalon rightMaster, rightSlave, leftMaster, leftSlave;
	
	//Shooting Motors
	CANTalon flyWheel, flyWheel2, hood, turret, lifter, kicker, intakeArm, wheelOfDoom;
	
	//Climbing Motors
	CANTalon climber, climber2;
	
	//Gear Motors
	CANTalon gearArm, gearIntake;
	
	//Solenoids
	Solenoid backSolenoid, frontSolenoid, armSolenoid, gearSolenoid;
	
	//Gyro
	ADXRS450_Gyro horzGyro;
	
	//Compressor
	Compressor compressor;
	
	Preferences pref;
	
	AnalogInput hoodEnc;
	
	NetworkTable table;
	Timer autoTime,turretTime;
	
	double intakeSpeed,robotSpeed,visionKP,autoP;
	boolean camNum = false,autonEncReset = true;
	double flyWheelRPM = 2900;
	
	enum BallsOut{
		reset,
		forward,
		turn,
		tacticalReload,
		stopAndShoot
	};
	BallsOut ballsOutAuton;
	
	enum GearAuto{
		forward,
		turn,
		pivot,
		place,
		drop,
		moveAndShoot,
		dontMove
	}
	GearAuto gearBallAuto;
	
	enum DriveForward{
		drive,
		stop
	}
	DriveForward driveAuto;
	Timer gearTime;
	
	double ballsOutForward1Left,ballsOutForward1Right,ballsOutTurn,ballsOutTurret,directionMultiplier;
	double gearsTurn,gearForwardLeft,gearForwardRight;
	
	
	
	SendableChooser<String> side = new SendableChooser<>();
	SendableChooser<String> auton = new SendableChooser<>();
	SendableChooser<String> position = new SendableChooser<>();

	@Override
	public void robotInit() {
		
		gearTime = new Timer();
		
		side.addDefault("Red Alliance", redSide);
		side.addObject("Blue Alliance", blueSide);
		SmartDashboard.putData("Auton Side Selection", side);
		
		auton.addDefault("40 Ball", ballsOut);
		auton.addObject("10 Ball Gear Auto", gearShoot);
		SmartDashboard.putData("Auton Selection",auton);
		
		position.addDefault("Position 1", pos1);	
		position.addObject("Position 2", pos2);
		position.addObject("Position 3", pos3);
		SmartDashboard.putData("Position Selection",position);
		
		
		pref = Preferences.getInstance();
		
		hoodEnc = new AnalogInput(1);
		
		autoTime = new Timer();
		turretTime = new Timer();
		
		//Gear Manipulator
		gearArm = new CANTalon(8);
		gearArm.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		gearIntake = new CANTalon(7);
		
		//Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);
		
		
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
		flyWheel = new CANTalon(10);
		flyWheel.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		flyWheel.reverseSensor(false);
		flyWheel2 = new CANTalon(11);
		flyWheel2.changeControlMode(TalonControlMode.Follower);
		flyWheel2.set(10);
		
		hood = new CANTalon(9);
		turret= new CANTalon(6);
		turret.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		lifter = new CANTalon(5);
		kicker = new CANTalon(4);
		intakeArm =  new CANTalon(13);
		wheelOfDoom = new CANTalon(12);
		
		//flyWheel Settings
		flyWheel.configNominalOutputVoltage(+0.0f,  -0.0f);
		flyWheel.configPeakOutputVoltage(+12.0f, -12.0f);
		
		flyWheel.setProfile(0);  
		flyWheel.setP(flyP); //0.1
		flyWheel.setI(flyI); //0.0004895
		flyWheel.setD(flyD); //0.5
		flyWheel.setF(0);
		
		//Climbing Motors
		climber = new CANTalon(2);
		climber2 = new CANTalon(3);
		climber2.changeControlMode(TalonControlMode.Follower);
		climber2.set(2);
		
		
		//Solenoids
		frontSolenoid = new Solenoid(2);
		backSolenoid = new Solenoid(1);
		armSolenoid = new Solenoid(3);
		gearSolenoid = new Solenoid(0);
		
		//Gyro
	    horzGyro = new ADXRS450_Gyro();
	    
	    table = NetworkTable.getTable("SmartDashboard"); 
		table.putBoolean("Camera_Type", camNum);
	   
		UsbCamera camera1;
	    camera1 = CameraServer.getInstance().startAutomaticCapture();
	    camera1.setResolution(320, 240);
	    camera1.setFPS(30);
	    horzGyro.calibrate();
		smartDash();
	}

	@Override
	public void autonomousInit() {
		
		sideSelected = (String) side.getSelected();
		autoSelected = (String) auton.getSelected();		
		positionSelected = (String) position.getSelected();
		
		System.out.println(sideSelected + " " +autoSelected + " "+ positionSelected);
		
		switch(sideSelected){
			case redSide:
				switch(autoSelected){
					case ballsOut:
						switch(positionSelected){
							case pos1: //close hopper bin red side
								ballsOutForward1Left = -2240;
								ballsOutForward1Right = 2334;
								ballsOutTurn = 90;
								ballsOutTurret = -8150;
								directionMultiplier = 1;
								smartDash();
								turret.setEncPosition(0);
								
								frontSolenoid.set(true);//This deploys omni if not already deployed.
								backSolenoid.set(false); // ^	
								turretTime.start();
	
								ballsOutAuton = BallsOut.reset;
								
								break;
							case pos2: //far hopper bin red side
								ballsOutForward1Left = -2391;
								ballsOutForward1Right = 2484;
								ballsOutTurn = 90;
								ballsOutTurret = 8350;
								directionMultiplier = -1;
								smartDash();
								turret.setEncPosition(0);
								
								frontSolenoid.set(true);//This deploys omni if not already deployed.
								backSolenoid.set(false); // ^	
								turretTime.start();
	
								ballsOutAuton = BallsOut.reset;
								break;
							case pos3:
								break;
						}
						break;
					case gearShoot:
						switch(positionSelected){
						case pos1:
							gearsTurn =-56;
							gearForwardLeft =-7470;
							gearForwardRight =7670;
							horzGyro.reset();
							leftMaster.setEncPosition(0);
							rightMaster.setEncPosition(0);
							gearArm.setEncPosition(0);
							gearTime.reset();
							gearTime.start();
							frontSolenoid.set(false);
							backSolenoid.set(false);
							gearBallAuto = GearAuto.forward;
							break;
						case pos2:
							horzGyro.reset();
							leftMaster.setEncPosition(0);
							rightMaster.setEncPosition(0);
							gearArm.setEncPosition(0);
							gearTime.reset();
							gearTime.start();
							frontSolenoid.set(false);
							backSolenoid.set(false);
							gearForwardLeft = -3370;
							gearForwardRight =3470;
							gearBallAuto = GearAuto.forward;
							break;
						case pos3:
							break;
					}
						break;
				}
				break;
			case blueSide:
				switch(autoSelected){
					case ballsOut:
						switch(positionSelected){
						case pos1:
							ballsOutForward1Left = -1991;
							ballsOutForward1Right = 2084; //flip sign
							ballsOutTurn = -90;
							ballsOutTurret = -8250;
							directionMultiplier = 1;
							smartDash();
							turret.setEncPosition(0);
							
							frontSolenoid.set(true);//This deploys omni if not already deployed.
							backSolenoid.set(false); // ^	
							turretTime.start();
	
							ballsOutAuton = BallsOut.reset;
							break;
						case pos2:
							ballsOutForward1Left = -3891;
							ballsOutForward1Right = 3984;
							ballsOutTurn = -90;
							ballsOutTurret =-3100;
							directionMultiplier = 1;
							smartDash();
							turret.setEncPosition(0);
							
							frontSolenoid.set(true);//This deploys omni if not already deployed.
							backSolenoid.set(false); // ^	
							turretTime.start();
	
							ballsOutAuton = BallsOut.reset;
							break;
						case pos3:
							break;
				}
					break;
				case gearShoot:
					switch(positionSelected){
					case pos1:
						gearsTurn =56;
						gearForwardLeft =-7470;
						gearForwardRight =7670;
						horzGyro.reset();
						leftMaster.setEncPosition(0);
						rightMaster.setEncPosition(0);
						gearArm.setEncPosition(0);
						gearTime.reset();
						gearTime.start();
						frontSolenoid.set(false);
						backSolenoid.set(false);
						gearBallAuto = GearAuto.forward;
						break;
					case pos2:
						horzGyro.reset();
						leftMaster.setEncPosition(0);
						rightMaster.setEncPosition(0);
						gearArm.setEncPosition(0);
						gearTime.reset();
						gearTime.start();
						frontSolenoid.set(false);
						backSolenoid.set(false);
						gearBallAuto = GearAuto.forward;
						break;
					case pos3:
						break;
				}
					break;
			}
			break;
	}
	
	
	}

	@Override
	public void autonomousPeriodic() {
		
		if(autoSelected.equalsIgnoreCase("Gear and Shoot Auto")){
			Gear10();
		}
		else{
			if(autoSelected.equalsIgnoreCase("40 Ball Auto")){
				BallsOut();
			}
		}
		
		/*
		switch(driveAuto){
		case drive:
			leftMaster.set(-.7);
			rightMaster.set(.7);	
			if(autoTime.get() >= 2){
				leftMaster.set(0);
				rightMaster.set(0);
				driveAuto = DriveForward.stop;
			}
			break;
		case stop:
			leftMaster.set(0);
			rightMaster.set(0);
			break;
		}
	*/
	}

	@Override
	public void teleopInit() {
		flyWheel.setEncPosition(0);
		rightMaster.setEncPosition(0);
		rightSlave.setEncPosition(0);
		leftMaster.setEncPosition(0);
		leftSlave.setEncPosition(0);
		turret.setEncPosition(0);
		//frontSolenoid.set(true); //This deploys omni if not already deployed.
		//backSolenoid.set(true);	 // ^
		//armSolenoid.set(true);   //Deploys arm if not already deployed.
		
	}
	
	@Override
	public void teleopPeriodic() {
		//System.out.println("rightMaster:  " + rightMaster.getEncPosition() + "leftMaster:  " + leftMaster.getEncPosition() + "rightSlave:  " + rightSlave.getEncPosition() + "leftSlave:  " + leftSlave.getEncPosition());
		if(driver.getRawButton(leftTrigger)){
			climber.set(1);
		}else{
			climber.set(0);
		}
		System.out.println("Turret Enc: "+ turret.getEncPosition());
		gearManipulator();
		shooter();
		hood();
		drivebase();
		smartDash();
		//System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());
		//System.out.println("P: " + flyP + " I: " + flyI + " D: " + flyD+ " F: " +flyWheel.getF());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	void BallsOut(){
		switch(ballsOutAuton){
		case reset:
			rightMaster.setEncPosition(0);
			rightSlave.setEncPosition(0);
			leftMaster.setEncPosition(0);
			leftSlave.setEncPosition(0);
			turret.setEncPosition(0);
			gearArm.setEncPosition(0);
			horzGyro.reset();
			if(leftMaster.getEncPosition() == 0){
				ballsOutAuton = BallsOut.forward;
			}
			break;
		case forward:
			//turret.set(-normPID(ballsOutTurret,turret.getEncPosition(),0.0011,0));
			leftMaster.set(-normPID(ballsOutForward1Left,leftMaster.getEncPosition(),0.00217,0));
			rightMaster.set(-normPID(ballsOutForward1Right,rightMaster.getEncPosition(),0.00217,0));
			System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());
			
			
			
			if(Math.abs(leftMaster.getEncPosition()) >= ((Math.abs(ballsOutForward1Left))-6) || (Math.abs(rightMaster.getEncPosition()) >= ((Math.abs(ballsOutForward1Right))-4))){
				leftMaster.set(0);
				rightMaster.set(0);
				rightMaster.setEncPosition(0);
				rightSlave.setEncPosition(0);
				leftMaster.setEncPosition(0);
				leftSlave.setEncPosition(0);
				
				ballsOutAuton = BallsOut.turn;
			}

			break;
		case turn:
				leftMaster.set(-normPID(ballsOutTurn,horzGyro.getAngle(),0.007,0));
				rightMaster.set(-normPID(ballsOutTurn,horzGyro.getAngle(),0.007,0));
				System.out.println("WE MADE IT TO TURN" + "Gyro: " + horzGyro.getAngle());
			
				
				
				if(Math.abs(horzGyro.getAngle()) >= (Math.abs(ballsOutTurn) -3)){
					leftMaster.set(0);
					rightMaster.set(0);
					rightMaster.setEncPosition(0);
					rightSlave.setEncPosition(0);
					leftMaster.setEncPosition(0);
					leftSlave.setEncPosition(0);
					autoTime.start();

					ballsOutAuton = BallsOut.tacticalReload;
				}

			 break;
		case tacticalReload:
			
			flyWheel.changeControlMode(TalonControlMode.Speed);
			flyWheel.set(-3300);

			lifter.set(-1);
			kicker.set(-1);
			System.out.println("TACTICAL RELOAD");
			gearArm.set(normPID(0,gearArm.getEncPosition(),0.00049,0));
			System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());
			
			leftMaster.set(.8 * directionMultiplier);
			rightMaster.set(-.8 * directionMultiplier);
			turret.set(normPID(ballsOutTurret,turret.getEncPosition(),0.0011,0));
	
			
			if(autoTime.get() >= .9 && turret.getEncPosition() >= (ballsOutTurret-50)){
				leftMaster.set(0);
				rightMaster.set(0);
				turret.set(0);
				ballsOutAuton = BallsOut.stopAndShoot;
			}
			break;
		case stopAndShoot:
			leftMaster.set(.2* directionMultiplier);
			rightMaster.set(-.2* directionMultiplier);
			turret.set(normPID(ballsOutTurret,turret.getEncPosition(),0.0011,0));

			wheelOfDoom.set(-1);
			flyWheel.changeControlMode(TalonControlMode.Speed);
			flyWheel.set(-3000);
			lifter.set(-1);
			kicker.set(-1);
			break;	
	}
		
	}
	
	void Gear10(){
		switch(gearBallAuto){
		case forward:
			leftMaster.set(.4 - normPID(0,horzGyro.getAngle(),0.00217,0));
			rightMaster.set(-.45 + normPID(0,horzGyro.getAngle(),0.00317,0));
			smartDash();
			if(leftMaster.getEncPosition() <= gearForwardLeft && rightMaster.getEncPosition() >=gearForwardRight ){
				leftMaster.set(0);
				rightMaster.set(0);
				gearBallAuto = GearAuto.turn;
			}
			
			break;
		case turn:
			leftMaster.set(-normPID(gearsTurn,horzGyro.getAngle(),.02,0));
			rightMaster.set(-normPID(gearsTurn,horzGyro.getAngle(),.02,0));
			if(horzGyro.getAngle() >= (Math.abs(gearsTurn)-24)){
				leftMaster.set(0);
				rightMaster.set(0);
				gearBallAuto = GearAuto.pivot;
			}
			break;
		case pivot:
			gearArm.set(-normPID(-250,gearArm.getEncPosition(),0.00097,0));
			if(gearArm.getEncPosition() <= -230){
				gearArm.set(0);
				leftMaster.setEncPosition(0);
				rightMaster.setEncPosition(0);
				gearBallAuto = GearAuto.place;
			}
			break;
		case place:
			leftMaster.set(.2 - normPID(0,horzGyro.getAngle(),0.00217,0));
			rightMaster.set(-.25 + normPID(0,horzGyro.getAngle(),0.00317,0));
			gearArm.set(-normPID(-240,gearArm.getEncPosition(),0.00097,0));

			smartDash();
			if(gearTime.get()>=6.9){
				leftMaster.set(0);
				rightMaster.set(0);
				gearBallAuto = GearAuto.drop;
			}
			break;
		
		case drop:
			if(gearTime.get() >= 7.3){
			gearArm.set(-normPID(-1100,gearArm.getEncPosition(),0.00097,0));
			}
			leftMaster.set(-.25 - normPID(0,horzGyro.getAngle(),0.00217,0));
			rightMaster.set(.3 + normPID(0,horzGyro.getAngle(),0.00317,0));
			if(leftMaster.getEncPosition() >= 2000 && rightMaster.getEncPosition() <= -2100){
				leftMaster.set(0);
				rightMaster.set(0);
				gearArm.set(0);
			}
			
			break;
		case moveAndShoot:
			leftMaster.set(-.2 - normPID(0,horzGyro.getAngle(),0.00217,0));
			rightMaster.set(.25 + normPID(0,horzGyro.getAngle(),0.00317,0));
			if(leftMaster.getEncPosition() >= 1500 && rightMaster.getEncPosition() <= -1400){
				leftMaster.set(0);
				rightMaster.set(0);
			}
			break;
		case dontMove:
			leftMaster.set(0);
			rightMaster.set(0);
			break;
		}
	}
	void drivebase(){
		double speed = deadBand(-driver.getY());
		double turn = deadBand(-driver.getZ());
		
		leftMaster.set((-speed + turn));
		rightMaster.set((speed + turn));
		
		
		
		//Solenoids
		if(driver.getRawButton(leftBumper)){
			frontSolenoid.set(true); //omni
			backSolenoid.set(true);  //omni
		}
		
		if(driver.getRawButton(rightBumper)){
			frontSolenoid.set(false);//traction
			backSolenoid.set(false);//traction
		}
		
		
		
		if(driver.getRawButton(1)){
			rightMaster.setEncPosition(0);
			rightSlave.setEncPosition(0);
			leftMaster.setEncPosition(0);
			leftSlave.setEncPosition(0);
		}
	
		
	}
	
	void hood(){
		//System.out.println("Hood Position: " +hoodEnc.getValue());
		
		
		if(oper.getPOV() == 0){ //key shot
			hood.set(-hoodPID(700,hoodEnc.getValue()));
			flyWheelRPM = 3115;
		}
		if(oper.getPOV() == 90){ //gear closest to boiler
			hood.set(-hoodPID(742,hoodEnc.getValue()));
			flyWheelRPM = 3550;
		}
		if(oper.getPOV() == 180){//gear 2
			hood.set(-hoodPID(725,hoodEnc.getValue()));
			flyWheelRPM = 3450;		
		}
		if(oper.getPOV() == 270){ //hopper
			hood.set(-hoodPID(740,hoodEnc.getValue()));
			flyWheelRPM = 2900;
		}
		if(oper.getRawButton(rightBumper)){
			hood.set(.4);
		}
		else{
			if(oper.getRawButton(rightTrigger)){
				hood.set(-.4);
			}
			else{
				hood.set(0);
			}
		}
	
	}
	
	
	void shooter(){
		if(oper.getRawButton(buttonOption)){
			turret.set(.5);
		}
		else{
			if(oper.getRawButton(buttonShare)){
				turret.set(-.5);
			}
			else{
				turret.set(0);
			}
		}
		
		
		intakeArm.set(-(deadBand(oper.getRawAxis(5))));
		smartDash();
		//shooting
	
		if(oper.getRawButton(buttonTriangle)){
			flyWheel.changeControlMode(TalonControlMode.Speed);
			flyWheel.set(-flyWheelRPM);

			lifter.set(-1);
			kicker.set(-1);
			smartDash();
			
		}else{
			flyWheel.changeControlMode(TalonControlMode.PercentVbus);
			flyWheel.set(0);
			lifter.set(0);
			kicker.set(0);
		}
		if(oper.getRawButton(buttonCircle)){
			wheelOfDoom.set(-1);													
		}else{
			if(oper.getRawButton(rightAnalog)){
				wheelOfDoom.set(1);
			}else{
				wheelOfDoom.set(0);
			}
		}
		
		
		smartDash();
		
		
		if(oper.getRawButton(touchpad)){
			turret.set(visionPID(175,table.getNumber("COG_X",0),visionKP));
		}
			
		}	
	
	void gearManipulator(){
		
		if(oper.getRawButton(buttonSquare)){ //up position for gear arm
			gearArm.set(normPID(-1300,gearArm.getEncPosition(),0.00049,0));
		}
		else{
			if(oper.getRawButton(buttonX)){ //deliver position for gear arm
				gearArm.set(normPID(-977,gearArm.getEncPosition(),0.00069,0));
			}
			else{
				if(gearArm.getEncPosition() <= -1400){ 
					gearArm.set(normPID(-1350,gearArm.getEncPosition(),0.00317,0));
				}
				else{
					gearArm.set(0.4*deadBand(oper.getY())); //left stick operator
				}
			}
		}
		if(oper.getRawButton(leftBumper)){
			gearIntake.set(-.75);
		}
		else{
			if(oper.getRawButton(leftTrigger)){
				gearIntake.set(.75);
			}
			else{
				gearIntake.set(0);
			}
		}
		if(oper.getRawButton(buttonPS)){
			gearArm.setEncPosition(0);
			turret.setEncPosition(0);
		}
	}
	double visionPID(double target,double position,double kP){
		double error = target - position;
		double speed = error *kP;
		return speed;
		
	}
	
	void smartDash(){
		
		SmartDashboard.putNumber("Speed", flyWheel.getSpeed());
		SmartDashboard.putNumber("COG_X", table.getNumber("COG_X",216));
		SmartDashboard.putNumber("COG_Y", table.getNumber("COG_Y",216));
		SmartDashboard.putNumber("Gear Arm Encoder", gearArm.getEncPosition());
		SmartDashboard.putNumber("Hood Encoder", hoodEnc.getValue());
		SmartDashboard.putNumber("Gyro Angle", horzGyro.getAngle());
		SmartDashboard.putNumber("Front Left Encoder", leftMaster.getEncPosition());
		SmartDashboard.putNumber("Back Left Encoder", leftSlave.getEncPosition());
		SmartDashboard.putNumber("Front Right Encoder", rightMaster.getEncPosition());
		SmartDashboard.putNumber("Back Right Encoder", rightSlave.getEncPosition());
		SmartDashboard.putNumber("Turret Spin Encoder", turret.getEncPosition());
	

		System.out.println(table.getNumber("COG_X",216));
		System.out.println(table.getNumber("COG_Y",216));
		
		flyP = pref.getDouble("P",0.008);
		flyI = pref.getDouble("I",0.000023);
		flyD = pref.getDouble("D",0.07);
		flyF = pref.getDouble("F",10);
		
		intakeSpeed = pref.getDouble("Intake", 1);
		robotSpeed = pref.getDouble("Speed",1);
		wheelRPM = pref.getDouble("RPM", 3240);
		hoodValue = pref.getDouble("Hood", 650);
		visionKP = pref.getDouble("VisionP", 0.00217);
		autoP = pref.getDouble("AutonGyroP", 0.0099);
		
		autoForwardLeft = pref.getDouble("ForwardLeft", 27);
		autoForwardRight = pref.getDouble("ForwardRight", 27);
		
		
		flyWheel.setP(flyP); //0.1
		flyWheel.setI(flyI); //0.0004895
		flyWheel.setD(flyD); //0.5
		flyWheel.setF(flyF);
	}
	
	void autoSmartDash(){
		flyP = pref.getDouble("P",0.035);
		flyI = pref.getDouble("I",0.0000376);
		flyD = pref.getDouble("D",0.35);
		
		flyWheel.setP(flyP); //0.1
		flyWheel.setI(flyI); //0.0004895
		flyWheel.setD(flyD); //0.5
		
		visionKP = pref.getDouble("VisionP", 0.00217);
		autoP = pref.getDouble("AutonGyroP", 0.00217);
		
	}
	double normPID(double target, double position, double pVal, double iVal){
		double PIDerror = target - position;
		double pOutput = (PIDerror*pVal);
		//double iOutput = (PIDerror*iVal);
		double speed = (pOutput);
		if(Math.abs(PIDerror) < 5){
			speed = 0;
		}
		
		return speed;
	}
	
	double deadBand(double joyStick){
		if(joyStick > -0.4 && joyStick < 0.4){
			joyStick = 0;
		}
		return joyStick;
	}
	
	double hoodPID(double target, double position){
		double error = target - position;
		double speed = error * 0.17;
		
		if(speed < 0.5 && error >10){
			speed*=2;
		}
		if(speed > 1){
			speed = 1;
		}
		
		return speed;
	}
	
	
}