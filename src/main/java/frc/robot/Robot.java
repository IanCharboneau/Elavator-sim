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
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
  private final MotionMagicExpoVoltage m_mmReq = new MotionMagicExpoVoltage(0);
  private final XboxController m_joystick = new XboxController(0);

  private final TalonFXSimState m_simState = m_fx.getSimState();




  private final ElevatorSim m_elevatorSim = 
  new ElevatorSim(
    DCMotor.getKrakenX60Foc(2),
    11.7, // Gear ratio
    12.0, 
    Inches.of(1.0*3).in(Meter), 
    0.3,
    2.0,
    true,
    0.3,
    0.0,
    5.0

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
    m_elevatorSim.setInput(m_simState.getMotorVoltage()); 
     



  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 11.7; // 12.8 rotor rotations per mechanism rotation

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
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

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





    m_mechanisms.update(m_fx.getPosition(), m_fx.getVelocity());
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

    m_fx.setControl(m_mmReq.withPosition(leftY * 10).withSlot(0));
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
