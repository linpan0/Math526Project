package frc.robot.drivetrain.gyro

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
  class GyroIOInputs : LoggableInputs {
    var rotation2d = Rotation2d()
    var angularVelocity = 0.0

    override fun toLog(table: LogTable) {
      table.put("Rotation2d", rotation2d)
      table.put("Angular Velocity", angularVelocity)
    }

    override fun fromLog(table: LogTable) {
      rotation2d = table.get("Rotation2d", rotation2d)[0]
      angularVelocity = table.get("Angular Velocity", angularVelocity)
    }
  }

  fun updateInputs(inputs: GyroIOInputs)

  fun reset()
}