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


/**x
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
	
	double flywheelPID;
	
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
	
	
	
	double wheelRPM, hoodAngle, flyKp, flyKi, flyKd;
	
	//Drive Motors
	CANTalon rightMaster, rightSlave, leftMaster, leftSlave;
	
	//Shooting Motors
	CANTalon flyWheelMaster, flyWheelSlave, hoodMotor, turretMotor, elevatorMotor, wheelGrabberMotor, ballIntakeMotor, wheelOfDoomMotor;
	
	//Climbing Motors
	CANTalon climberMaster, climberSlave;
	
	//Gear Motors
	CANTalon gearArmMotor, gearIntakeMotor;
	
	//Solenoids
	Solenoid backWheelSolenoid, frontWheelSolenoid, ballIntakeSolenoid;
	
	//Gyro
	ADXRS450_Gyro horzGyro;
	
	//Compressor
	Compressor compressor;
	
	Preferences pref;
	
	AnalogInput hoodEnconder;
	
	NetworkTable table;
	Timer autoTime,turretTime;
	
	double flyWheelRPM = 2900;
	
	SendableChooser<String> side = new SendableChooser<>();
	SendableChooser<String> auton = new SendableChooser<>();
	SendableChooser<String> position = new SendableChooser<>();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//Gear Manipulator
		gearArmMotor = new CANTalon(8);
		gearArmMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		gearIntakeMotor = new CANTalon(7);
		
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
		flyWheelMaster = new CANTalon(10);
		flyWheelMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		flyWheelMaster.reverseSensor(false);
		flyWheelSlave = new CANTalon(11);
		flyWheelSlave.changeControlMode(TalonControlMode.Follower);
		flyWheelSlave.set(10);
		
		hoodMotor = new CANTalon(9);
		turretMotor= new CANTalon(6);
		turretMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		elevatorMotor = new CANTalon(5);
		wheelGrabberMotor = new CANTalon(4);
		ballIntakeMotor =  new CANTalon(13);
		wheelOfDoomMotor = new CANTalon(12);
		
		//flyWheel Settings
		flyWheelMaster.configNominalOutputVoltage(+0.0f,  -0.0f);
		flyWheelMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		
		flyWheelMaster.setProfile(0);  
		flyWheelMaster.setP(flyKp); //0.1
		flyWheelMaster.setI(flyKi); //0.0004895
		flyWheelMaster.setD(flyKd); //0.5
		flyWheelMaster.setF(0);
		
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
	    
	/*    table = NetworkTable.getTable("SmartDashboard"); 
		table.putBoolean("Camera_Type", camNum);
	   
		UsbCamera camera1;
	    camera1 = CameraServer.getInstance().startAutomaticCapture();
	    camera1.setResolution(320, 240);
	    camera1.setFPS(30);
	    horzGyro.calibrate();
		smartDash(); */
	}
@Override
	public void autonomousInit() {
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
default:
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//System.out.println("rightMaster:  " + rightMaster.getEncPosition() + "leftMaster:  " + leftMaster.getEncPosition() + "rightSlave:  " + rightSlave.getEncPosition() + "leftSlave:  " + leftSlave.getEncPosition());
		if(driver.getRawButton(leftTrigger)){
			climberMaster.set(1);
		}else{
			climberMaster.set(0);
		}
		System.out.println("Turret Enc: "+ turretMotor.getEncPosition());
//		gearManipulator();
		shooter();
		drivebase();
	//	smartDash();
		//System.out.println("Front Left: " + leftMaster.getEncPosition() + " Front Right: " + rightMaster.getEncPosition() + " Back Left: " + leftSlave.getEncPosition() + " Back Right: " + rightSlave.getEncPosition() + " Gyro: "+ horzGyro.getAngle());
		//System.out.println("P: " + flyP + " I: " + flyI + " D: " + flyD+ " F: " +flyWheel.getF());
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
	
	void shooter(){
				if(oper.getRawButton(buttonShare)){
					turretMotor.set(-.5);
				}
				else{
					turretMotor.set(0);
				}
		
			ballIntakeMotor.set(-(deadBand(oper.getRawAxis(leftBumper))));
			//shooting
		
			if(oper.getRawButton(buttonTriangle)){
				flyWheelMaster.changeControlMode(TalonControlMode.Speed);
				flyWheelMaster.set((-normPID(flyWheelRPM, flyWheelMaster.getEncVelocity(), flyKp, flyKi)));
				elevatorMotor.set(-1);
				wheelGrabberMotor.set(-1);
				
			}else{
				flyWheelMaster.changeControlMode(TalonControlMode.PercentVbus);
				flyWheelMaster.set(0);
				elevatorMotor.set(0);
				wheelGrabberMotor.set(0);
			}
			if(oper.getRawButton(buttonCircle)){
				wheelOfDoomMotor.set(-1);													
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
		double pOutput = (PIDerror*flyKp);
		double iOutput = (PIDerror*flyKi);
		double speed = (pOutput+iOutput);
		if(Math.abs(PIDerror) < 1){
			speed = 0;
		}

		return speed;
	}
}