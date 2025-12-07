package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Telemetry Panels Style", group = "Dev")
public class TestTelemetryPanelsStyle extends OpMode {

    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        // Access the singleton's telemetry manager
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        panelsTelemetry.addData("Key", "Value");
        panelsTelemetry.addData("Key2", new CustomObject(10));

        panelsTelemetry.debug("String 1", "String 2", "String 3", new CustomObject(10));

        // Update both Panels and FTC telemetry
        panelsTelemetry.update(telemetry);
    }

    public static class CustomObject {
        private int number;
        public CustomObject(int number) { this.number = number; }
        @Override
        public String toString() { return "Number: " + number; }
    }
}