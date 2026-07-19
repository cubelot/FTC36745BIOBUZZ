import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class Sensors {
    private DistanceSensor distance;

    public void init(HardwareMap hwMap) {
        distance = hwMap.get(DistanceSensor.class, "sensorDistance");
    }

    public double getDistance() {
        return distance.getDistance(DistanceUnit.CM);
    }
}