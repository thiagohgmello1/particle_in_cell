

class Material:
    color = 255

    def __init__(self, relax_time, permittivity=1, permeability=1, color=None):
        self.relax_time = relax_time
        self.permittivity = permittivity
        self.permeability = permeability
        if not color:
            color = Material.color
        self.color = (color, color, color)

        if Material.color >= 10:
            Material.color -= 10
        else:
            Material.color = 0
