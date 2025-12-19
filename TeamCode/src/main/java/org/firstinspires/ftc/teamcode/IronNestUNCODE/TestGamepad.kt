package org.firstinspires.ftc.teamcode.IronNestUNCODE

import com.bylazar.gamepad.PanelsGamepad
// Use this import for the telemetry manager
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Test Gamepad", group = "Dev")
class TestGamepad : OpMode() {
    private val g1 = PanelsGamepad.firstManager
    private val g2 = PanelsGamepad.secondManager

    // In version 1.0.12, use the static instance from PanelsTelemetry
    private val panelsTelemetry = PanelsTelemetry.telemetry

    override fun init() {
        panelsTelemetry.debug("Gamepad Test Initialized")
        // Sends data to BOTH the Dashboard (Port 8001) and Driver Hub
        panelsTelemetry.update(telemetry)
    }

    override fun loop() {
        // Bridges physical hardware to the Panels data stream
        val combinedG1 = g1.asCombinedFTCGamepad(gamepad1)
        val combinedG2 = g2.asCombinedFTCGamepad(gamepad2)

        panelsTelemetry.debug("==== Gamepad 1 Buttons ====")
        panelsTelemetry.debug("A: ${combinedG1.a}")
        panelsTelemetry.debug("B: ${combinedG1.b}")

        // This call is CRITICAL: It packages the gamepad state and
        // broadcasts it to the graphical interface.
        panelsTelemetry.update()
    }
}
