import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp

public class DistanceTest {
TestBenchDistance bench = new TestBechDistance();

@Override
public void init() {
    bench.init(hardwareMap);
}
@Override
public void loop() {
    telemetry.addData("Distance",  bench.getDistance());
}
}
