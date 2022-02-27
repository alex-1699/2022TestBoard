/*
CONTROLS!!!!!!!!!!!!!!!!!!!!!!!!!!!!

k so. u gotta click some buttons to do some things. nothing is automated

DRIVER JOYSTICK:
move the joystick (x and y) to move the robot (hopefully)
click button 5 (top left face button) to toggle both climber solenoids

OPERATOR JOYSTICK:
hold trigger to hold the HOPPER STOPPER down
press button 2 (thumb button) to toggle the INTAKE ARM solenoid
button 11 runs SHOOTER
hold button 12 to run the intake motor
press button 9 to toggle THE HOOD
*/


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  public Joystick driveJoystick;
  public Joystick opJoystick;
  public TalonSRX m_shooterPort;
  public TalonSRX m_shooterStar;
  public TalonSRX m_intakerMan;
  public double throt;
  //drive talons
  public TalonFX m_portDriveFX1, m_portDriveFX2, m_portDriveFX3, m_starDriveFX1, m_starDriveFX2, m_starDriveFX3;
  private DoubleSolenoid intakeSolenoid, hopperStopper, shooterAngleSolenoid, climberSolenoidPort;
  
  public double throtSpd;
  
  private void toggleSolenoid(final DoubleSolenoid solenoid){
    if(solenoid.get() == DoubleSolenoid.Value.kForward){
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }else{
        solenoid.set(DoubleSolenoid.Value.kForward);
    }}

  @Override
  public void robotInit() {

    driveJoystick = new Joystick(Constants.kDriveJoystickPort);
    opJoystick = new Joystick(Constants.kOperatorJoystickPort);
    //shooter motors
    m_shooterPort = new TalonSRX(Constants.kPortShooterPort);
    m_shooterStar = new TalonSRX(Constants.kStarShooterPort);

    //drive mototrz (port)
    m_portDriveFX1 = new TalonFX(Constants.kPortDrivePort1);
    m_portDriveFX2 = new TalonFX(Constants.kPortDrivePort2);
    m_portDriveFX3 = new TalonFX(Constants.kPortDrivePort3);
    //drive mortoe (starboard)
    m_starDriveFX1 = new TalonFX(Constants.kStarDrivePort1);
    m_starDriveFX2 = new TalonFX(Constants.kStarDrivePort2);
    m_starDriveFX3 = new TalonFX(Constants.kStarDrivePort3);

    //inake motor!!!
    m_intakerMan = new TalonSRX(Constants.kIntakeHoppPort);

    //solenoids <3
    intakeSolenoid = new DoubleSolenoid(Constants.kIntakeSolenoidModulePort, PneumaticsModuleType.CTREPCM, Constants.kIntakeSolenoidForwardPort, Constants.kIntakeSolenoidReversePort);
    hopperStopper = new DoubleSolenoid(Constants.kFlopperSolenoidModulePort, PneumaticsModuleType.CTREPCM, Constants.kFlopperSolenoidForwardPort, Constants.kFlopperSolenoidReversePort);
    shooterAngleSolenoid = new DoubleSolenoid(Constants.kShooterAngleSolenoidModulePort, PneumaticsModuleType.CTREPCM, Constants.kShooterAngleSolenoidForwardPort, Constants.kShooterAngleSolenoidReversePort);
    climberSolenoidPort = new DoubleSolenoid(Constants.kPortClimberModulePort, PneumaticsModuleType.CTREPCM, Constants.kPortClimberForwardPort, Constants.kPortClimberReversePort);


    m_portDriveFX2.follow(m_portDriveFX1);
    m_portDriveFX3.follow(m_portDriveFX1);

    m_starDriveFX2.follow(m_starDriveFX1);
    m_starDriveFX3.follow(m_starDriveFX1);

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    //sets the throttle thingy which makes robot go slo
    throt = (-driveJoystick.getThrottle() + 1) / 2;

    //DRIVE!!!!!!!!!
    runArcadeDrive(driveJoystick.getX(), -driveJoystick.getY());

    //INTAKE! INTAKE! INTAKE...
    if (opJoystick.getRawButton(12)){
      m_intakerMan.set(TalonSRXControlMode.PercentOutput, -0.5);
    } else {
      m_intakerMan.set(TalonSRXControlMode.PercentOutput, 0);
    }
    if (opJoystick.getRawButtonPressed(2)) {
      toggleSolenoid(intakeSolenoid);
    }
    //HOPPA STOPPA
    if (opJoystick.getRawButtonPressed(1)) {
      toggleSolenoid(hopperStopper);
    }
    if (opJoystick.getRawButtonReleased(1)) {
      hopperStopper.toggle();
      toggleSolenoid(hopperStopper);
    }

    //run both shooter boys when u click button 11 on operator joystick
    if (opJoystick.getRawButton(11)){
      m_shooterPort.set(TalonSRXControlMode.PercentOutput, -0.3);
      m_shooterStar.set(TalonSRXControlMode.PercentOutput, 0.3);  
    } else {
      m_shooterPort.set(TalonSRXControlMode.PercentOutput, 0);
      m_shooterStar.set(TalonSRXControlMode.PercentOutput, 0);
    }
    
    //climby climby
    if (driveJoystick.getRawButtonPressed(5)) {
      toggleSolenoid(climberSolenoidPort);
    }

    //shooter angle UwU
    if (opJoystick.getRawButtonPressed(9)) {
      toggleSolenoid(shooterAngleSolenoid);
    }

  }
  protected void runArcadeDrive(double throttle, double rotate) {
    double portOutput = 0.0;
    double starOutput = 0.0;

    throttle = Math.copySign(throttle * throttle, throttle);
    rotate = Math.copySign(rotate * rotate, rotate);

    double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(rotate)), throttle);

    if (throttle >= 0.0) {
        // First quadrant, else second quadrant
        if (rotate >= 0.0) {
            portOutput = maxInput;
            starOutput = throttle - rotate;
        } else {
            portOutput = throttle + rotate;
            starOutput = maxInput;
        }
    } else {
        // Third quadrant, else fourth quadrant
        if (rotate >= 0.0) {
            portOutput = throttle + rotate;
            starOutput = maxInput;
        } else {
            portOutput = maxInput;
            starOutput = throttle - rotate;
        }
    }

    //actaully set the drive motors
    m_portDriveFX1.set(TalonFXControlMode.PercentOutput, portOutput * throt);

    m_starDriveFX1.set(TalonFXControlMode.PercentOutput, starOutput * throt);
}
}
