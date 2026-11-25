package first.minolib.hardware;

/**
 * Represents a CAN device identifier with bus name and device number. Used to uniquely identify and
 * configure CAN-based devices on the robot.
 */

public class CANDeviceID {
    // 2027: SystemCore has no "rio" bus. Default to SystemCore bus 0 to match Constants.kSystemCoreBus0.
    public static final String kRIOCANbusName = "can_s0";
    public int deviceNumber;
    public final String CANbusName;

    public CANDeviceID(final int deviceNumber, final String CANbusName) {
        this.deviceNumber = deviceNumber;
        this.CANbusName = CANbusName;
    }
  
    public CANDeviceID(final int deviceNumber) {
        this(deviceNumber, kRIOCANbusName);
    }

    public void newDeviceID(final int deviceNumber) {
        this.deviceNumber = deviceNumber;
    }

    @Override
    public String toString() {
        return "[" + CANbusName + " " + deviceNumber + "]";
    }
  }