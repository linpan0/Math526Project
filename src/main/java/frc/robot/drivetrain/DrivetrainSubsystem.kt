package frc.robot.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.drivetrain.commands.TankDriveCommand
import frc.robot.drivetrain.gyro.GyroIO
import frc.robot.utils.LoggedTunableNumber
import org.littletonrobotics.junction.Logger

class DrivetrainSubsystem(private val drivetrainIO: DrivetrainIO, private val gyroIO: GyroIO) : SubsystemBase() {
  private val drivetrainInputs = DrivetrainIO.DrivetrainIOInputs()
  private val gyroInputs = GyroIO.GyroIOInputs()

  init {
    resetDriveEncoders()
  }

  private val leftDistancePID = PIDController(25.0, 0.0, 1.0)
  private val rightDistancePID = PIDController(25.0, 0.0, 1.0)
  private val distanceP = LoggedTunableNumber("Distance/P", 25.0)
  private val distanceI = LoggedTunableNumber("Distance/I", 0.0)
  private val distanceD = LoggedTunableNumber("Distance/D", 1.0)
  private val distanceSetpoint = LoggedTunableNumber("Distance/Setpoint", 0.5)

  override fun periodic() {
    drivetrainIO.updateInputs(drivetrainInputs)
    gyroIO.updateInputs(gyroInputs)

    Logger.processInputs("Drivetrain", drivetrainInputs)
    Logger.processInputs("Gyro", gyroInputs)

    LoggedTunableNumber.ifChanged(
      distanceP.hashCode() + distanceI.hashCode() + distanceP.hashCode(),
      { leftDistancePID.setPID(it[0], it[1], it[2]) },
      distanceP, distanceI, distanceD
    )

    LoggedTunableNumber.ifChanged(
      distanceP.hashCode() + distanceI.hashCode() + distanceP.hashCode(),
      { rightDistancePID.setPID(it[0], it[1], it[2]) },
      distanceP, distanceI, distanceD
    )
  }

  fun setDesiredVolts(distanceSetpoint: Double) {
    leftDistancePID.setpoint = distanceSetpoint
    rightDistancePID.setpoint = distanceSetpoint
    Logger.recordOutput("Drivetrain/DistanceSetpoint", distanceSetpoint)

    val leftVolts = MathUtil.clamp(leftDistancePID.calculate(getLeftDistance()), -12.0, 12.0)
    val rightVolts = MathUtil.clamp(rightDistancePID.calculate(getRightDistance()), -12.0, 12.0)
    setVolts(leftVolts, rightVolts)
  }

  fun atDesiredDistance() = leftDistancePID.atSetpoint() && rightDistancePID.atSetpoint()

  fun getDistanceSetpoint() = distanceSetpoint.get()

  fun resetDistancePID() {
    leftDistancePID.reset()
    rightDistancePID.reset()
  }

  fun tankDrive(leftSpeed: Double, rightSpeed: Double) = drivetrainIO.tankDrive(leftSpeed, rightSpeed)

  fun setVolts(leftVolts: Double, rightVolts: Double) {
    drivetrainIO.setLeftVolts(leftVolts)
    drivetrainIO.setRightVolts(rightVolts)
  }

  fun stop() = setVolts(0.0, 0.0)

  fun getLeftDistance() = drivetrainInputs.leftDistance

  fun getRightDistance() = drivetrainInputs.rightDistance

  fun resetGyro() = gyroIO.reset()

  fun resetDriveEncoders() = drivetrainIO.resetDriveEncoders()

  fun setDefaultCommand(controller: CommandPS5Controller) {
    defaultCommand = TankDriveCommand(this, controller)
  }
}