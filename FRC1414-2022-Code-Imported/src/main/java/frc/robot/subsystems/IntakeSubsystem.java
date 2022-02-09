// package frc.robot.subsystems;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class IntakeSubsystem extends SubsystemBase{
//   public final CANSparkMax frontIntakeMotor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
//   public final CANSparkMax backIntakeMotor = new CANSparkMax(Constants.BACK_INTAKE_ID, MotorType.kBrushless);

//   private final DoubleSolenoid frontIntakePiston = new DoubleSolenoid(Constants.PCM_ID,
//   Constants.FRONT_INTAKE_FORWARD_ID, Constants.FRONT_INTAKE_RETRACTED_ID);

//   private final DoubleSolenoid backIntakePiston = new DoubleSolenoid(Constants.PCM_ID,
//   Constants.BACK_INTAKE_FORWARD_ID, Constants.BACK_INTAKE_RETRACTED_ID);

//   private Value frontPistonState = Constants.INTAKE_PISTON_CLOSED;
//   private Value backPistonState = Constants.INTAKE_PISTON_CLOSED;


//   // private final Compressor compressor = new Compressor(Constants.PCM_ID);

//   public IntakeSubsystem(){
//     this.frontIntakeMotor.setIdleMode(IdleMode.kBrake);
//     this.frontIntakeMotor.setInverted(true);

//     this.backIntakeMotor.setIdleMode(IdleMode.kBrake);
//     this.backIntakeMotor.setInverted(true);
//   }

//   public void setFrontIntakeSpeed(double speed) {
//     this.frontIntakeMotor.set(speed);
//   }

//   public void setBackIntakeSpeed(double speed) {
//     this.backIntakeMotor.set(speed);
//   }

//   public Value getFrontPistonState() {
//     return this.frontPistonState;
//   }

//   public void toggleFrontIntake() {
//     if (getFrontPistonState() == Constants.INTAKE_PISTON_OPEN) {
//       this.frontIntakePiston.set(Constants.INTAKE_PISTON_CLOSED);
//       this.frontPistonState = Constants.INTAKE_PISTON_CLOSED;
//     } else if (getFrontPistonState() == Constants.INTAKE_PISTON_CLOSED) {
//       this.frontIntakePiston.set(Constants.INTAKE_PISTON_OPEN);
//       this.frontPistonState = Constants.INTAKE_PISTON_OPEN;
//     }
//   }

//   public void setFrontPistonForward() {
//     this.frontIntakePiston.set(Constants.INTAKE_PISTON_OPEN);
//     this.frontPistonState = Constants.INTAKE_PISTON_OPEN;
//   }

//   public void setFrontPistonReverse() {
//     this.frontIntakePiston.set(Constants.INTAKE_PISTON_CLOSED);
//     this.frontPistonState = Constants.INTAKE_PISTON_CLOSED;
//   }

//   public Value getBackPistonState() {
//     return this.frontPistonState;
//   }

//   public void toggleBackIntake() {
//     if (getBackPistonState() == Constants.INTAKE_PISTON_OPEN) {
//       this.backIntakePiston.set(Constants.INTAKE_PISTON_CLOSED);
//       this.backPistonState = Constants.INTAKE_PISTON_CLOSED;
//     } else if (getBackPistonState() == Constants.INTAKE_PISTON_CLOSED) {
//       this.backIntakePiston.set(Constants.INTAKE_PISTON_OPEN);
//       this.backPistonState = Constants.INTAKE_PISTON_OPEN;
//     }
//   }

//   public void setBackPistonForward() {
//     this.backIntakePiston.set(Constants.INTAKE_PISTON_OPEN);
//     this.backPistonState = Constants.INTAKE_PISTON_OPEN;
//   }

//   public void setBackPistonReverse() {
//     this.backIntakePiston.set(Constants.INTAKE_PISTON_CLOSED);
//     this.backPistonState = Constants.INTAKE_PISTON_CLOSED;
//   }

// }// END OF CLASS