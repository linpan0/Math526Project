package frc.robot.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.drivetrain.commands.TankDriveCommand
import frc.robot.drivetrain.gyro.GyroIO
import frc.robot.utils.LoggedTunableNumber
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class DrivetrainSubsystem(private val drivetrainIO: DrivetrainIO, private val gyroIO: GyroIO) : SubsystemBase() {
  private val drivetrainInputs = DrivetrainIO.DrivetrainIOInputs()
  private val gyroInputs = GyroIO.GyroIOInputs()

  init {
    resetDriveEncoders()
  }
  /**
   * Distance PID
   */
  private val leftDistancePID = PIDController(25.0, 0.0, 1.0)
  private val rightDistancePID = PIDController(25.0, 0.0, 1.0)
  private val distanceP = LoggedTunableNumber("Distance/P", 25.0)
  private val distanceI = LoggedTunableNumber("Distance/I", 0.0)
  private val distanceD = LoggedTunableNumber("Distance/D", 1.0)
  private val distanceSetpoint = LoggedTunableNumber("Distance/Setpoint", 0.5)

  /**
   * State space
   */

  /**
   * States: [left velocity, right velocity]ᵀ
   * Inputs: [left voltage, right voltage]ᵀ
   * Outputs: [left velocity, right velocity]ᵀ
   */
  private val leftWheelPlant: LinearSystem<N2, N2, N2> = LinearSystemId.createDrivetrainVelocitySystem(
    DCMotor.getRomiBuiltIn(1),
    0.5,
    DrivetrainConstants.WHEEL_DIAMETER_METERS / 2,
    0.13,
    0.00032,
    1.0
  )

  private val leftKalmanFilter = KalmanFilter(
    Nat.N2(),
    Nat.N2(),
    leftWheelPlant,
    VecBuilder.fill(3.0, 3.0),
    VecBuilder.fill(0.01, 0.01),
    0.02
  )

  private val leftLQR = LinearQuadraticRegulator(
    leftWheelPlant,
    VecBuilder.fill(0.3, 0.3),
    VecBuilder.fill(12.0, 12.0),
    0.02
  )

  private val leftLoop = LinearSystemLoop(
    leftWheelPlant,
    leftLQR,
    leftKalmanFilter,
    7.5,
    0.02
  )

  override fun periodic() {
    drivetrainIO.updateInputs(drivetrainInputs)
    gyroIO.updateInputs(gyroInputs)

    Logger.processInputs("Drivetrain", drivetrainInputs)
    Logger.processInputs("Gyro", gyroInputs)

    LoggedTunableNumber.ifChanged(
      distanceP.hashCode() + distanceI.hashCode() + distanceP.hashCode(),
      { leftDistancePID.setPID(it[0], it[1], it[2]); rightDistancePID.setPID(it[0], it[1], it[2]) },
      distanceP, distanceI, distanceD
    )
  }

  fun setDesiredVoltsV1(distanceSetpoint: Double) {
    leftDistancePID.setpoint = distanceSetpoint
    rightDistancePID.setpoint = distanceSetpoint
    Logger.recordOutput("Drivetrain/DistanceSetpoint", distanceSetpoint)

    val leftVolts = MathUtil.clamp(leftDistancePID.calculate(getLeftDistance()), -7.5, 7.5)
    val rightVolts = MathUtil.clamp(rightDistancePID.calculate(getRightDistance()), -7.5, 7.5)
    setVolts(leftVolts, rightVolts)
  }

  fun setDesiredVoltsV2(distanceSetpoint: Double) {
    if (atDesiredDistanceV2()) return
    leftLoop.nextR = VecBuilder.fill(distanceSetpoint, distanceSetpoint)
    leftLoop.correct(VecBuilder.fill(drivetrainInputs.leftVelocity, drivetrainInputs.rightVelocity))
    leftLoop.predict(0.20)
    val nextVolts = leftLoop.getU(0)
    setVolts(nextVolts, nextVolts)
  }

  fun atDesiredDistanceV1() = leftDistancePID.atSetpoint() && rightDistancePID.atSetpoint()

  fun atDesiredDistanceV2(): Boolean {
    return abs(drivetrainInputs.leftDistance - distanceSetpoint.get()) < 0.03 &&
      abs(drivetrainInputs.rightDistance - distanceSetpoint.get()) < 0.03
  }

  fun getDistanceSetpoint() = distanceSetpoint.get()

  fun resetLoopState() = leftLoop.reset(VecBuilder.fill(0.0, 0.0))

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