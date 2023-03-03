// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //DriveConfig
  private DifferentialDrive m_myRobot;

  private static final int rightDeviceId = 4;
  private static final int rightDeviceId2 = 5;
  private static final int rightDeviceId3 = 6;
  private static final int leftDeviceId = 1;
  private static final int leftDeviceId2 = 2;
  private static final int leftDeviceId3 = 3;

  private CANSparkMax m_rightMotor;
  private CANSparkMax m_rightMotor2;
  private CANSparkMax m_rightMotor3;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_leftMotor3;

  //Forearm
  private static final int forearmID = 7;

  private CANSparkMax forearmMotor;

  //Wrist
  private static final int wristID = 8;

  private CANSparkMax wristMotor;

  //Intake
  private static final int intakeMotorID = 9;
  private static final int intakeMotorID2 = 10;

  private CANSparkMax intakeMotor;
  private CANSparkMax intakeMotor2;
  
  //JoystickConfig
  private final XboxController driveStick = new XboxController(0);
  private final Joystick opStick = new Joystick(1);
  private final Joystick arcadeStick = new Joystick(2);
  private final Joystick m_leftStick = new Joystick(3);
  private final Joystick m_rightStick = new Joystick(4);

  //Encoder Wrist
  private SparkMaxPIDController wristPID;
  private RelativeEncoder wristEncoder;
  //public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  //AutoBalance
  private AHRS m_gyro;

  //Limit Switch
  DigitalInput limit = new DigitalInput(0);
  DigitalInput limit2 = new DigitalInput(1);

  //Pnumaticts
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1,2);  

  //Encoder Drivetrain
  private RelativeEncoder m_rightMotorEncoder;
  
  private RelativeEncoder m_leftMotorEncoder;
 

  public SparkMaxPIDController m_LPidController;
 
  public SparkMaxPIDController m_RPidController;

  double averagePosition;

  double revs;

  double distance;

  private PIDController m_pidController;

  double kP = 0.1;
  double setpoint = 0;

  //double kGyroKp = 0.1;

  public double convertEncounterCountsToAxis(double encoderCounts) {
    return encoderCounts / (50.0 / 12.0) * (60.0 / 14.0);


  }
  

  @Override
  public void robotInit() {

    //DriveConfig
    m_rightMotor = new CANSparkMax(rightDeviceId, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(rightDeviceId2, MotorType.kBrushless);
    m_rightMotor3 = new CANSparkMax(rightDeviceId3, MotorType.kBrushless);
    m_leftMotor = new CANSparkMax(leftDeviceId, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(leftDeviceId2, MotorType.kBrushless);
    m_leftMotor3 = new CANSparkMax(leftDeviceId3, MotorType.kBrushless);

    MotorControllerGroup right = new MotorControllerGroup(m_rightMotor, m_rightMotor2, m_rightMotor3);
    MotorControllerGroup left = new MotorControllerGroup(m_leftMotor, m_leftMotor2, m_leftMotor3);

    m_rightMotor.setInverted(true);
    m_rightMotor2.setInverted(true);
    m_rightMotor3.setInverted(true);

    m_myRobot = new DifferentialDrive(left, right);

    //Autobalance
    //m_gyro = new AHRS(Port.kMXP);

    //Forearm
    forearmMotor = new CANSparkMax(forearmID, MotorType.kBrushless);

    //Wrist 
    wristMotor = new CANSparkMax(wristID, MotorType.kBrushless);

    //Intake
    intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushed);
    intakeMotor2 = new CANSparkMax(intakeMotorID2, MotorType.kBrushed);

    //Encoder
    wristPID = wristMotor.getPIDController();

    wristEncoder = wristMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    wristPID.setFeedbackDevice(wristEncoder);

    //Encoder Drivetrain
    m_pidController = new PIDController(kP, 0.0, 0.0);
    m_rightMotorEncoder = m_rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  
    m_leftMotorEncoder = m_leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    

    m_RPidController = m_rightMotor.getPIDController();
    
    m_LPidController = m_leftMotor.getPIDController();
   

    m_RPidController.setFeedbackDevice(m_rightMotorEncoder);
  
    m_LPidController.setFeedbackDevice(m_leftMotorEncoder);
   

  }

  

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Distance", distance);

    SmartDashboard.putData(m_pidController);

    kP = m_pidController.getP();

  }

  @Override
  public void autonomousInit() {
    m_leftMotorEncoder.setPosition(0);
    m_rightMotorEncoder.setPosition(0);
  }

  @Override
  public void autonomousPeriodic() {


    double speedLimit = 0.4;
    double turnLimit = 0.4;


    m_myRobot.setSafetyEnabled(false);

    setpoint = 10;

    averagePosition = (m_leftMotorEncoder.getPosition() + m_rightMotorEncoder.getPosition()) / 2;

    revs = convertEncounterCountsToAxis(averagePosition) / 42.0;

    distance = revs * 6 * Math.PI;

    double error = setpoint - distance;

    double outputSpeed = kP * error;

    m_myRobot.arcadeDrive(outputSpeed * speedLimit, 0);


   

  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    //Drive Code
    //Half Speed Setup
    double speedLimit = 0.6;
    double turnLimit = 0.6;

    if(m_leftStick.getRawButton(1)){
      speedLimit = 0.9;
      turnLimit = 0.9;
    }

    //Arcade Xbox
    //m_myRobot.arcadeDrive(speedLimit*-driveStick.getLeftY(), turnLimit*-driveStick.getRightX());

    //Tank Xbox
    //m_myRobot.tankDrive(speedLimit*-driveStick.getLeftY(), turnLimit*-driveStick.getRightY());

    //Arcade Joystick
    m_myRobot.arcadeDrive(speedLimit*-m_leftStick.getY(), turnLimit*-m_rightStick.getX());
    

    //Tank Joystick
    //m_myRobot.tankDrive(speedLimit*-m_leftStick.getY(), speedLimit*-m_rightStick.getY());

    //Wrist
    if (opStick.getRawButton(9)){
      //Down
      wristMotor.set(0.6);
    }
      else if (opStick.getRawButton(10)){
        //Up
        wristMotor.set(-0.6);
      }
          else{
            wristMotor.set(0);
          }

  //Limit Switches
  if(!limit.get() && (opStick.getRawButton(10))){
      wristMotor.set(0);
  }

  if(!limit2.get()){
    wristMotor.set(0);
  }

  //Intake
  double outtakeSpeed = opStick.getThrottle();
  if (opStick.getRawButton(1)){
   //Intake
   intakeMotor.set(-0.6);
   intakeMotor2.set(0.6);
  }
    else if (opStick.getRawButton(6)){
      //Outake
      intakeMotor.set(0.8);
      intakeMotor2.set(-0.8);
    }
      else if(opStick.getRawButton(4)){
        intakeMotor.set(0.5);
        intakeMotor2.set(-0.5);
      }
        else{
          intakeMotor.set(0);
          intakeMotor2.set(0);
        }

    //Forearm
    if (opStick.getRawButton(7)){
      forearmMotor.set(0.2);
    }
      else if(opStick.getRawButton(8)){
        forearmMotor.set(-0.2);
      }
        else{
          forearmMotor.set(0);
        }

    //Pnumatics
    // Set Forward button
    /*if (opStick.getRawButton(11)) {
      //Close
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    // Set Reverse button
    if (opStick.getRawButton(12)) {
      //Open
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }*/

    //Encoders
    SmartDashboard.putNumber("Wrist Encoder Postion", wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Encoder Velocity", wristEncoder.getPosition());

    /*if(wristEncoder.getPosition()>32 && opStick.getRawButton(7)){
      wristMotor.set(0.4);
    }
      else if(wristEncoder.getPosition()<32 && opStick.getRawButton(7)){
        wristMotor.set(0.4);
      }
        else if(wristEncoder.getPosition()!=32 && opStick.getRawButton(7)){
          wristMotor.set(0);
        }
          else{
            wristMotor.set(0);
          }*/

    //Limelight      
   
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
