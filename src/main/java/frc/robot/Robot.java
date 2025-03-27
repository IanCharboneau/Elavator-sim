// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.sim.PhysicsSim;

import edu.wpi.first.wpilibj.simulation.BatterySim;






/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final TalonFX m_fx = new TalonFX(1, "canivore");
  private final MotionMagicExpoTorqueCurrentFOC m_mmReq = new MotionMagicExpoTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC m_tcReq = new PositionTorqueCurrentFOC(0);
  
  private final XboxController m_joystick = new XboxController(0);

  private TalonFXSimState m_simState = m_fx.getSimState();

  public final double k_gear_ratio = 11.7;
  public final double k_spool_radius = Inches.of(3).in(Meter);






  private final ElevatorSim m_elevatorSim = 
  new ElevatorSim(
    DCMotor.getKrakenX60Foc(2),
    k_gear_ratio, // Gear ratio
    12.0, 
    k_spool_radius, 
    0.0,
    1.676,
    true,
    0.0,
    0.0,
    0.0

  );

  

  private int m_printCount = 0;

  private final Mechanisms m_mechanisms = new Mechanisms();

  @Override
  public void simulationInit() {
    // PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
    
    m_simState.setSupplyVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(0));
    


    


  }

  @Override
  public void simulationPeriodic() {
    // PhysicsSim.getInstance().run();

    m_simState = m_fx.getSimState();
    m_simState.setSupplyVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    var motor_voltage = m_simState.getMotorVoltageMeasure();


    m_elevatorSim.setInputVoltage(motor_voltage.in(Volts));
    m_elevatorSim.update(0.020);

    m_simState.setRawRotorPosition(m_elevatorSim.getPositionMeters()*k_gear_ratio/(k_spool_radius*3.141592));
    m_simState.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond()*k_gear_ratio/(k_spool_radius*3.141592));
    
     



  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = k_gear_ratio*3.5; // gear ratio and spool size

    // Choose MotionMagicExpo kV and kA.
    // Use the default values scaled with the sensor ratio.
    // Increase these values for a "softer" response
    // Note these two configs are always in V (Volts) even when using
    // MotionMagicExpoDutyCycle or MotionMagicExpoTorqueCurrentFOC
    var newMotionMagicExpo_kV = Volts.per(Rotations.per(Second))
        .ofNative(0.12 * fdb.SensorToMechanismRatio); // 12 V / 100 RPS_motor * 12.8 (mechanism) rotations per motor
                                                      // rotation => 1.536 V / RPS (mechanism)
    var newMotionMagicExpo_kA = Volts.per(Rotations.per(Second).per(Second))
        .ofNative(0.10 * fdb.SensorToMechanismRatio); // 0.1 V / RPS^2 * 12.8 (mechanism) rotations per motor rotation
                                                      // => 1.28 V / RPS^2 (mechanism)

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100))
      // setup MotionMagicExpo kV and kA
      .withMotionMagicExpo_kV(newMotionMagicExpo_kV)
      .withMotionMagicExpo_kA(newMotionMagicExpo_kA);

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.0; // Add 0.0 A output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 A output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 A output
    slot0.kP = 0; // A position error of 0.2 rotations results in 12 A output
    slot0.kG = 20;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 A output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void robotPeriodic() {
    if (++m_printCount >= 10) {
      m_printCount = 0;
      System.out.println("Pos: " + m_elevatorSim.getPositionMeters());
      System.out.println("Vel: " + m_elevatorSim.getVelocityMetersPerSecond());
      System.out.println("Current: " + m_elevatorSim.getCurrentDrawAmps());
      System.out.println();
    }





    m_mechanisms.update(m_elevatorSim.getPositionMeters(), m_elevatorSim.getVelocityMetersPerSecond());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    /* Deadband the joystick */
    double leftY = m_joystick.getLeftY();
    if (Math.abs(leftY) < 0.1) leftY = 0;

    // m_fx.setControl(m_mmReq.withPosition(leftY * 1).withSlot(0));
    m_fx.setControl(m_tcReq.withPosition(leftY * 1).withSlot(0));
    if (m_joystick.getBButton()) {
      m_fx.setPosition(Rotations.of(1));
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
