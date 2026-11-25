package first.minolib.hardware;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import first.minolib.io.CANBusInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

/**
 * The MinoCANBus class represents a CAN bus interface for the robot. It provides functionality to
 * log and update the status of the CAN bus.
 *
 * <p>It includes an inner static class CANBusInputs which holds the status and various metrics of
 * the CAN bus.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * MinoCANBus canBus = new MinoCANBus("can_s0");
 * canBus.updateInputs();
 * }</pre>
 *
 * <p>Constructor Summary:
 *
 * <ul>
 *   <li>{@link #MinoCANBus(String canbusName)} - Initializes the CAN bus with the specified name
 *       ("can_s0" to "can_s4" for SystemCore's built-in buses, or a CANivore name / "*").
 * </ul>
 *
 * <p>Method Summary:
 *
 * <ul>
 *   <li>{@link #updateInputs()} - Updates the CAN bus inputs and logs the current status.
 * </ul>
 *
 * <p>Inner Class:
 *
 * <ul>
 *   <li>{@link MinoCANBus.CANBusInputs} - Represents the inputs for a CAN bus.
 * </ul>
 */
public class MinoCANBus {
  private final String canbusName;
  private final String loggingName;
  private final CANBus canBus;

  private final CANBusInputsAutoLogged inputs = new CANBusInputsAutoLogged();

  private final Debouncer canivoreConnectedDebouncer = new Debouncer(0.5, DebounceType.RISING); // TODO: Change this to use a LoggedTunableNumber
  private final Alert canivoreErrorAlert;

  public MinoCANBus(final String canbusName) {
    this.canbusName = canbusName;
    loggingName = "Inputs/CANBus [" + canbusName + "]";

    // 2027: CANBus no longer takes a hoot replay file. Use HootReplay if log replay is needed later.
    canBus = new CANBus(this.canbusName);

    // 2027: Alert needs an id that is unique per bus (we have several buses now); AlertType.kError -> Level.HIGH
    canivoreErrorAlert = new Alert("CANBusError [" + canbusName + "]", "CAN bus " + canbusName + " error detected, robot may not be controllable.", Level.HIGH);
  }

  public void updateInputs() {
    CANBusStatus status = canBus.getStatus();

    inputs.status = status.Status;
    inputs.busUtilization = status.BusUtilization;
    inputs.busOffCount = status.BusOffCount;
    inputs.txFullCount = status.TxFullCount;
    inputs.REC = status.REC;
    inputs.TEC = status.TEC;
    inputs.isNetworkFD = canBus.isNetworkFD();
    Logger.processInputs(loggingName, inputs);

    boolean debouncedError = canivoreConnectedDebouncer.calculate(!status.Status.isOK() || status.REC > 0 || status.TEC > 0);
    canivoreErrorAlert.set(debouncedError);
  }

  public CANBus getParent() {
    return canBus;
  }

  public String getName() {
    return canBus.getName();
  }
}