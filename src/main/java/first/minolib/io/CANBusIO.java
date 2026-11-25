package first.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface CANBusIO {
    
    @AutoLog
    public static class CANBusInputs {
        public StatusCode status = StatusCode.OK;
        public double busUtilization = 0.0;
        public long busOffCount = 0; // 2027: CTRE changed this to long
        public long txFullCount = 0; // 2027: CTRE changed this to long
        public int REC = 0;
        public int TEC = 0;
        public boolean isNetworkFD = false;
    }
}