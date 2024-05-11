package frc.robot.drivetrain.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.romi.RomiGyro

class GyroIORomi : GyroIO {
  private val gyro = RomiGyro()

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.rotation2d = Rotation2d(gyro.angle)
    inputs.angularVelocity = gyro.rate
  }

  override fun reset() = gyro.reset()
}