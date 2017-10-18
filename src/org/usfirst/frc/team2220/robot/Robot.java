package org.usfirst.frc.team2220.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {

	CANTalon right1 = new CANTalon (6);
	CANTalon left1 = new CANTalon (8);
	CANTalon right2 = new CANTalon (15);
	CANTalon left2 = new CANTalon (2);

	double autoSpeed = 0.5;
	double speed = 0.3;
	
	AHRS ahrs;
	Joystick stick = new Joystick(0);
	
	
	public Robot(){
		try {
	          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
	          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
	          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
	          ahrs = new AHRS(SPI.Port.kMXP); 
	      } catch (RuntimeException ex ) {
	          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
	      }
	}
	
	public void robotInit(){
		//right1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);	
	}
	
	public void autonomous(){
		while(isOperatorControl() == false){
			right1.setPosition(0);
			left1.setPosition(0);
			right1.set(1);
			left1.set(1);
		}
	}
	
	public double deadzone(double val, double zone){
		double signum = Math.signum(val);
		return signum * Math.pow(Math.abs(val), 1.8);
	}
	
	public void operatorControl(){
		//right1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		//left1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		//right1.changeControlMode(CANTalon.TalonControlMode.Position);
		//left1.changeControlMode(CANTalon.TalonControlMode.Position);
		right1.changeControlMode(TalonControlMode.PercentVbus);
		left1.changeControlMode(TalonControlMode.PercentVbus);
		/*
		left1.configEncoderCodesPerRev(128);
		right1.configEncoderCodesPerRev(128);
		
		right1.setPosition(0);
		left1.setPosition(0);
		
		left1.setF(0.0);
		left1.setP(2.8);
		left1.setI(0.0);
		left1.setD(0.0);

		right1.setF(0.0);
		right1.setP(2.8);
		right1.setI(0.0);
		right1.setD(0.0);
		*/
		
		left2.changeControlMode(CANTalon.TalonControlMode.Follower);
		right2.changeControlMode(CANTalon.TalonControlMode.Follower);
		left2.set(left1.getDeviceID());
		right2.set(right1.getDeviceID());
		//StringBuilder _sb = new StringBuilder();
		//int _loops = 0;
		
		while (isOperatorControl() && isEnabled())
		{
			
			double leftAxis  = stick.getRawAxis(1) * -1;
			double rightAxis = stick.getRawAxis(5);
			leftAxis = deadzone(leftAxis, 0.06);
			rightAxis = deadzone(rightAxis, 0.06);
			if(stick.getRawButton(4)){
				right1.set(-speed);
				left1.set(speed);
			}
			else if(stick.getRawButton(1)){
				right1.set(speed);
				left1.set(-speed);
			}
			else if(stick.getRawButton(2)){
				right1.set(speed);
				left1.set(speed);
			}
			else if(stick.getRawButton(3)){
				right1.set(-speed);
				left1.set(-speed);
			}
			else
			{
				double scale = 1; //280 = 50%
				right1.set(rightAxis * scale);
				left1.set(leftAxis * scale);
			}
			
			SmartDashboard.putNumber("lError", left1.getError());
			SmartDashboard.putNumber("rError", right1.getError());
			/*
			SmartDashboard.putNumber("rSetpoint", right1.getSetpoint());
			SmartDashboard.putNumber("lSetpoint", left1.getSetpoint());
			*/
 			SmartDashboard.putNumber("left Voltage", left1.getOutputVoltage());
			SmartDashboard.putNumber("right Voltage", -right1.getOutputVoltage());
			/*
			
			double motorOutput = right1.getOutputVoltage() / right1.getBusVoltage();
			_sb.append("\trAxis:"+rightAxis);
			_sb.append("\tout:");
			_sb.append(motorOutput);
	        _sb.append("\tspd:");
	        _sb.append(right1.getSpeed() );
	        _sb.append("\terr:");
            _sb.append(right1.getClosedLoopError());
			//_sb.append("\tposR:"+ right1.getPosition());
			//_sb.append("\tposL:"+ left1.getPosition());
			if(++_loops >= 10) {
	        	_loops = 0;
	        	System.out.println(_sb.toString());
	        }
	        _sb.setLength(0);
	        */
			
			boolean zero_yaw_pressed = stick.getTrigger();
	          if ( zero_yaw_pressed ) {
	              ahrs.zeroYaw();
	          }

	          /* Display 6-axis Processed Angle Data                                      */
	          SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
	          SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
	          SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
	          
	          /* Display tilt-corrected, Magnetometer-based heading (requires             */
	          /* magnetometer calibration to be useful)                                   */
	          
	          SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
	          
	          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
	          SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

	          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
	          /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
	          
	          SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
	          SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

	          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
	          
	          SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
	          SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
	          SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
	          SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

	          /* Display estimates of velocity/displacement.  Note that these values are  */
	          /* not expected to be accurate enough for estimating robot position on a    */
	          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
	          /* of these errors due to single (velocity) integration and especially      */
	          /* double (displacement) integration.                                       */
	          
	          SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
	          SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
	          SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
	          SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
	          
	          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
	          /* NOTE:  These values are not normally necessary, but are made available   */
	          /* for advanced users.  Before using this data, please consider whether     */
	          /* the processed data (see above) will suit your needs.                     */
	          
	          SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
	          SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
	          SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
	          SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
	          SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
	          SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
	          SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
	          SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
	          SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
	          SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
	          
	          /* Omnimount Yaw Axis Information                                           */
	          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
	          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
	          SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
	          SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
	          
	          /* Sensor Board Information                                                 */
	          SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
	          
	          /* Quaternion Data                                                          */
	          /* Quaternions are fascinating, and are the most compact representation of  */
	          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
	          /* from the Quaternions.  If interested in motion processing, knowledge of  */
	          /* Quaternions is highly recommended.                                       */
	          SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
	          SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
	          SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
	          SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
	          
	          /* Connectivity Debugging Support                                           */
	          SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
	          SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
	        Timer.delay(0.005); //Motor update time
		}
	}
	
	public void test(){
		
	}
	
	public void ultrasonicSample() {
    	//double range = ultra.getRangeInches(); // reads the range on the ultrasonic sensor
    	//SmartDashboard.putNumber("Ultrasonic", range);
    }
	
}




