

class Material:
    color = 255

    def __init__(
            self,
            relax_time,
            fermi_vel,
            permittivity=1,
            permeability=1,
            e_mass=9.11e-31,
            e_charge=1.6e-19,
            color=None
    ):
        self.relax_time = relax_time
        self.permittivity = permittivity
        self.permeability = permeability
        self.electron_mass = e_mass
        self.electron_charge = e_charge
        self.fermi_velocity = fermi_vel
        if not color:
            color = Material.color
        self.color = (color, color, color)

        if Material.color >= 10:
            Material.color -= 10
        else:
            Material.color = 0
