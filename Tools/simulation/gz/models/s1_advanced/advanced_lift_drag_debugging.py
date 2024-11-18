import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import os


class AdvancedLiftDragSimulation:
    def __init__(self, sdf_file):
        self.params = {
            "CL0": 0.0,
            "CLa": 0.0,
            "CD0": 0.0,
            "Cem0": 0.0,
            "Cema": 0.0,
            "alpha_stall": 0.0,
            "Cema_stall": 0.0,
            "area": 0.0,
            "AR": 0.0,
            "eff": 1.0,
            "rho": 1.2041,  # Luftdichte (kg/m³)
            "mac": 0.0,
            "CD_fp_k1": 0.0,
            "CD_fp_k2": 0.0,
        }
        self.control_surfaces = []
        self.load_sdf(sdf_file)

    def load_sdf(self, sdf_file):
        tree = ET.parse(sdf_file)
        root = tree.getroot()

        plugin = root.find(".//plugin[@name='gz::sim::systems::AdvancedLiftDrag']")
        if plugin is None:
            raise ValueError("AdvancedLiftDrag plugin not found in the SDF file.")

        for key in self.params.keys():
            element = plugin.find(key)
            if element is not None:
                self.params[key] = float(element.text)

        control_surfaces = plugin.findall("control_surface")
        for surface in control_surfaces:
            name = surface.find("name").text
            direction = float(surface.find("direction").text)
            CL_ctrl = float(surface.find("CL_ctrl").text)
            CD_ctrl = float(surface.find("CD_ctrl").text)
            CY_ctrl = float(surface.find("CY_ctrl").text)
            self.control_surfaces.append({
                "name": name,
                "direction": direction,
                "CL_ctrl": CL_ctrl,
                "CD_ctrl": CD_ctrl,
                "CY_ctrl": CY_ctrl,
            })

    def calculate_forces_and_moments(self, alpha_deg, beta_deg, velocity, control_angles):
        alpha = np.radians(alpha_deg)
        beta = np.radians(beta_deg)

        CL, CD, CY = self._compute_coefficients(alpha, beta, control_angles)
        q = 0.5 * self.params["rho"] * velocity ** 2
        area = self.params["area"]

        lift = CL * q * area
        drag = CD * q * area
        sideforce = CY * q * area

        moments = self._compute_moments(alpha, beta, CL, CD, CY, velocity, control_angles)

        return lift, drag, sideforce, moments

    def _compute_coefficients(self, alpha, beta, control_angles):
        """Berechnet die aerodynamischen Koeffizienten."""
        CL = self._compute_CL(alpha, beta, control_angles)
        CD = self._compute_CD(alpha, CL, control_angles)
        CY = self._compute_CY(beta, control_angles)
        return CL, CD, CY

    def _compute_CL(self, alpha, beta, control_angles):
        """Berechnet den Auftriebskoeffizienten."""
        CL0 = self.params["CL0"]
        CLa = self.params["CLa"]
        alpha_stall = self.params["alpha_stall"]

        sigma = 1 / (1 + np.exp(-10 * (alpha - alpha_stall)))
        CL_prestall = CL0 + CLa * alpha
        CL_poststall = 2 * np.sign(alpha) * np.sin(alpha) ** 2 * np.cos(alpha)
        CL = (1 - sigma) * CL_prestall + sigma * CL_poststall

        # Kontrollflächen hinzufügen
        for i, control in enumerate(self.control_surfaces):
            control_angle = np.radians(control_angles[i])
            CL += control_angle * control["CL_ctrl"] * control["direction"]

        return CL

    def _compute_CD(self, alpha, CL, control_angles):
        """Berechnet den Widerstandskoeffizienten."""
        CD0 = self.params["CD0"]
        AR = self.params["AR"]
        eff = self.params["eff"]

        sigma = 1 / (1 + np.exp(-10 * (alpha - self.params["alpha_stall"])))
        CD_fp = 2 / (1 + np.exp(self.params["CD_fp_k1"] + self.params["CD_fp_k2"] * AR))

        CD_prestall = CD0 + (CL ** 2) / (np.pi * AR * eff)
        CD_poststall = abs(CD_fp * (0.5 - 0.5 * np.cos(2 * alpha)))
        CD = (1 - sigma) * CD_prestall + sigma * CD_poststall

        # Kontrollflächen-Widerstand hinzufügen
        for i, control in enumerate(self.control_surfaces):
            control_angle = np.radians(control_angles[i])
            CD += control_angle * control["CD_ctrl"] * control["direction"]

        return CD

    def _compute_CY(self, beta, control_angles):
        """Berechnet den Seitenkraftkoeffizienten."""
        CYb = self.params.get("CYb", 0.0)

        CY = CYb * beta
        for i, control in enumerate(self.control_surfaces):
            control_angle = np.radians(control_angles[i])
            CY += control_angle * control["CY_ctrl"] * control["direction"]

        return CY

    def _compute_moments(self, alpha, beta, CL, CD, CY, velocity, control_angles):
        """Berechnet Roll-, Nick- und Giermomente."""
        moments = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        q = 0.5 * self.params["rho"] * velocity ** 2
        mac = self.params["mac"]
        span = np.sqrt(self.params["area"] * self.params["AR"])

        # Aerodynamische Momente
        moments["roll"] = CY * q * self.params["area"] * span
        moments["pitch"] = CL * q * self.params["area"] * mac
        moments["yaw"] = CD * q * self.params["area"] * span

        # Kontrollflächenmomente hinzufügen
        for i, control in enumerate(self.control_surfaces):
            control_angle = np.radians(control_angles[i])
            moments["roll"] += control_angle * control.get("Cell_ctrl", 0) * control["direction"]
            moments["pitch"] += control_angle * control.get("Cem_ctrl", 0) * control["direction"]
            moments["yaw"] += control_angle * control.get("Cen_ctrl", 0) * control["direction"]

        return moments


# Beispielnutzung
if __name__ == "__main__":
    sdf_file = os.path.join(os.path.dirname(__file__), "model.sdf")
    simulation = AdvancedLiftDragSimulation(sdf_file)

    alpha = 10
    beta = 0
    velocity = 25
    control_angles = [5, 10, -5, 15]

    lift, drag, sideforce, moments = simulation.calculate_forces_and_moments(alpha, beta, velocity, control_angles)

    print(f"Lift: {lift:.2f} N, Drag: {drag:.2f} N, Sideforce: {sideforce:.2f} N")
    print(f"Moments: {moments}")
