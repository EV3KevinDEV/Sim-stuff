// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  private Field2d m_field = new Field2d();
 
  private Spark m_rightmotor = new Spark(4);
  private Spark m_rightmotor2 = new Spark(5);
  private Spark m_leftmotor = new Spark(6);
  private Spark m_leftmotor2 = new Spark(7);

  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);

  private EncoderSim leftEncoder = new EncoderSim(m_leftEncoder);
  private EncoderSim rightEncoder = new EncoderSim(m_rightEncoder);

  public AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim gyro = new AnalogGyroSim(m_gyro);

  MotorControllerGroup m_right = new MotorControllerGroup(m_rightmotor,m_rightmotor2);
  MotorControllerGroup m_left = new MotorControllerGroup(m_leftmotor, m_leftmotor2);
  //private DifferentialDrive drive = new DifferentialDrive(left, right);

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(), new Pose2d(2, 2, new Rotation2d()));

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
 
  public DifferentialDrivetrainSim m_drivesim = DifferentialDrivetrainSim.createKitbotSim(
     KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
     KitbotGearing.k10p71,        // 10.71:1
     KitbotWheelSize.kSixInch,     // 6" diameter wheels.
     null      // No measurement noise.
  );

  /** Creates a new ExampleSubsystem. */
  public Drivebase() {
    SmartDashboard.putData("Field", m_field);
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);
  }

  public void Rotate(double rotate){
    if(rotate > 0){
      TankDrive(-rotate, rotate);
    } else {
      TankDrive(rotate, -rotate);
    }

  }

  public DifferentialDriveWheelSpeeds getWheelSpeedsSim() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }
 

  @Override
  public void periodic() {
     m_odometry.update(m_gyro.getRotation2d(),
                    m_leftEncoder.getDistance(),
                    m_rightEncoder.getDistance());
  m_field.setRobotPose(m_odometry.getPoseMeters());
    // This method will be called once per scheduler run
  }

  public void controllerDrive(XboxController Xbox){
    m_left.set(-Xbox.getLeftY());
    m_right.set(-Xbox.getRightY());

  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }


  public void TankDrive(double left, double right){
    m_left.setVoltage(left);
    m_right.setVoltage(right);
    m_drive.feed();
  }

  public void resetGyroSim(){
    gyro.resetData();
  }

  public void resetGyro(){
    m_gyro.reset();
  }

  public void PoseReset(Pose2d yes){
    resetEncoders();
    m_odometry.resetPosition(yes, m_gyro.getRotation2d());
  }

  private void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  @Override
  public void simulationPeriodic() {
    m_drivesim.setInputs(m_left.get(), m_right.get());
  //  m_drivesim.setInputs(m_left.get()*RobotController.getInputVoltage(), m_right.get()*RobotController.getInputVoltage());
    m_drivesim.update(0.02);
    leftEncoder.setDistance(m_drivesim.getLeftPositionMeters());
    leftEncoder.setRate(m_drivesim.getLeftVelocityMetersPerSecond());
    rightEncoder.setDistance(m_drivesim.getRightPositionMeters());
    rightEncoder.setRate(m_drivesim.getRightVelocityMetersPerSecond());
    gyro.setAngle(-m_drivesim.getHeading().getDegrees());
    // This method will be called once per scheduler run during simulation
  }
}
