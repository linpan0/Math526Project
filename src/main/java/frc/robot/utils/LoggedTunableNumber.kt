package frc.robot.utils

import frc.robot.GlobalConstants
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

class LoggedTunableNumber(dashboardKey: String, private val defaultValue: Double = 0.0) {
  private val key = "$TABLE_KEY/$dashboardKey"
  private var dashboardNumber: LoggedDashboardNumber? = null
  private val lastHasChangedValues = hashMapOf<Int, Double>()

  init {
    if (GlobalConstants.TUNING_MODE) {
      dashboardNumber = LoggedDashboardNumber(key, defaultValue)
    }
  }

  fun get() = dashboardNumber?.get() ?: defaultValue

  fun hasChanged(id: Int): Boolean {
    val currentValue = get()
    val lastValue = lastHasChangedValues[id]
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues[id] = currentValue
      return true
    }

    return false
  }

  companion object {
    private const val TABLE_KEY = "TunableNumbers"

    fun ifChanged(id: Int, action: (DoubleArray) -> Unit, vararg tunableNumbers: LoggedTunableNumber) {
      tunableNumbers.filter { it.hasChanged(id) }.forEach { _ ->
        action(tunableNumbers.map { it.get() }.toDoubleArray())
      }
    }

    fun ifChanged(id: Int, action: Runnable, vararg tunableNumbers: LoggedTunableNumber) = ifChanged(id, { _: DoubleArray -> action.run() }, *tunableNumbers)
  }
}