package frc.robot;

// Default
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// Motor
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// Controller
import edu.wpi.first.wpilibj.Joystick;

// Sensor
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

// encoder
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import java.io.IOException;
// Other 
import java.lang.Math;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Timer;

// Auto


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //controllers  
  private Joystick joystick;
  private Joystick xbox;
  private Joystick pedal;

  //Motors

  ////////////////////////
  //chassis
  private WPI_TalonFX left1;
  private WPI_TalonFX left2;
  private WPI_TalonFX right1;
  private WPI_TalonFX right2;

  ///////////////////////

  // Motor Groupings
  private SpeedControllerGroup lSide;
  private SpeedControllerGroup rSide;

  // Drive Train
  private DifferentialDrive chassis;

  // Motor IDs

  //left side drive train
  private int LEFT1_ID = 2;
  private int LEFT2_ID = 3;

  //right side drive train
  private int RIGHT1_ID = 12;
  private int RIGHT2_ID = 13;

  // Control buttons and axis
  private int FOREWARD_BACKWARD_AXIS = 1;
  private int LEFT_RIGHT_AXIS = 2;

  private int DRIVING_SPEED = 3;

  private int TRIGGER = 1;
  private int THUMB_BUTTON = 2;

  private int LEFT_RIGHT_JOYCON = 0;
  private int FOREWARD_BACKWARD_JOYCON = 1;
  private int TRIGGER_JOYCON = 2;

  //accelerate and decelerate variables
  private boolean previousSpeed = false;
  private double brakePower = 0.3;
  private double joyconTopSpeed = 0;
  private double joyconTopSpeedTurn = 0;
  @Override
  public void robotInit() {
    //init
    joystick = new Joystick(0);
    xbox = new Joystick(1);
    pedal = new Joystick(2);

    left1 = new WPI_TalonFX(LEFT1_ID);
    left2 =  new WPI_TalonFX(LEFT2_ID);

    right1 = new WPI_TalonFX(RIGHT1_ID);
    right2 = new WPI_TalonFX(RIGHT2_ID);

    lSide = new SpeedControllerGroup(left1, left2);
    rSide = new SpeedControllerGroup(right1, right2);

    chassis = new DifferentialDrive(lSide, rSide);


  }


  private void drive() { // Drives the robot
    double topSpeed = joystick.getRawAxis(DRIVING_SPEED);
    topSpeed += 1;
    topSpeed /= 2;
    topSpeed = 1 - topSpeed;
    //System.out.println(topSpeed);
    chassis.arcadeDrive(-joystick.getRawAxis(FOREWARD_BACKWARD_AXIS)*topSpeed, -joystick.getRawAxis(LEFT_RIGHT_AXIS) * 0.66);
    if (-joystick.getRawAxis(FOREWARD_BACKWARD_AXIS) < 0) {
      previousSpeed = true;
    } else if(-joystick.getRawAxis(FOREWARD_BACKWARD_AXIS) > 0) {
      previousSpeed = false;
    }
    //System.out.println(previousSpeed);
  }

  private void drive_joycon() { // Drives the robot
    //System.out.println(topSpeed);
    chassis.arcadeDrive(-xbox.getRawAxis(FOREWARD_BACKWARD_JOYCON)*joyconTopSpeed, -xbox.getRawAxis(LEFT_RIGHT_JOYCON) * joyconTopSpeedTurn);
    if (-xbox.getRawAxis(FOREWARD_BACKWARD_JOYCON) < 0) {
      previousSpeed = true;
    } else if(-xbox.getRawAxis(FOREWARD_BACKWARD_JOYCON) > 0) {
      previousSpeed = false;
    }
    //System.out.println(previousSpeed);
  }

  private void set_topspeed(){
    if (xbox.getRawButton(7)){
      joyconTopSpeed = 0.75;
      joyconTopSpeedTurn = 0.66;
    } else if (xbox.getRawButton(3)){
      joyconTopSpeed = 0.5;
      joyconTopSpeedTurn = 0.56;
    } else if (xbox.getRawButton(1)){
      joyconTopSpeed = 0.6;
      joyconTopSpeedTurn = 0.60;
    } else if (xbox.getRawButton(5)){
      joyconTopSpeed = 0.4;
      joyconTopSpeedTurn = 0.5;
    }
  }

  private void brake(){ //brake button
    if (joystick.getRawButton(TRIGGER)){
      left1.setNeutralMode(NeutralMode.Brake);
      left2.setNeutralMode(NeutralMode.Brake);
      right1.setNeutralMode(NeutralMode.Brake);
      right2.setNeutralMode(NeutralMode.Brake);
    }else {
      left1.setNeutralMode(NeutralMode.Coast);
      left2.setNeutralMode(NeutralMode.Coast);
      right1.setNeutralMode(NeutralMode.Coast);
      right2.setNeutralMode(NeutralMode.Coast);
    }
  }

  private void brakeJoycon(){ //brake button
    if (xbox.getRawButton(9)){
      left1.setNeutralMode(NeutralMode.Brake);
      left2.setNeutralMode(NeutralMode.Brake);
      right1.setNeutralMode(NeutralMode.Brake);
      right2.setNeutralMode(NeutralMode.Brake);
    }else {
      left1.setNeutralMode(NeutralMode.Coast);
      left2.setNeutralMode(NeutralMode.Coast);
      right1.setNeutralMode(NeutralMode.Coast);
      right2.setNeutralMode(NeutralMode.Coast);
    }
  }

  private void decelerate(){
  if (joystick.getRawButton(THUMB_BUTTON)&& previousSpeed == false){
      chassis.arcadeDrive(-brakePower, 0);
  }else if (joystick.getRawButton(THUMB_BUTTON) && previousSpeed == true){
    chassis.arcadeDrive(brakePower, 0);
  }else{
    return;
  }
  }
  private void decelerateJoycon(){
    if (xbox.getRawAxis(TRIGGER_JOYCON) > 0&& previousSpeed == false){
        chassis.arcadeDrive(-brakePower, 0);
    }else if (xbox.getRawAxis(TRIGGER_JOYCON) > 0&& previousSpeed == true){
      chassis.arcadeDrive(brakePower, 0);
    }
    }
    /*
  private void emergencyBrake(){
    boolean loop = false;
    if (pedal.getRawButton(8)){
      loop = true;
      System.out.println("Button Pressed");
    }
    while (loop == true){
      if (xbox.getRawButton(1) && xbox.getRawButton(3)){
        loop = false;
      } else {
        left1.setNeutralMode(NeutralMode.Brake);
        left2.setNeutralMode(NeutralMode.Brake);
        right1.setNeutralMode(NeutralMode.Brake);
        right2.setNeutralMode(NeutralMode.Brake);
        chassis.arcadeDrive(0,0);
        System.out.println("Emergency Stop!");
      }
    }
    
    //3 and 1
  }
  */

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    chassis.arcadeDrive(0, 0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    drive_joycon();
    set_topspeed();
    decelerateJoycon();
    brakeJoycon();
    //emergencyBrake();
    
    /*
    decelerate();
    brake();
    drive();
    */
  }


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
