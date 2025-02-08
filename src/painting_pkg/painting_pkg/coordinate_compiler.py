class CoordinateCompiler:
    def __init__(self):
        pass

    def compile_coordinates(self, processed_curves):
        """Compiles a list of coordinate sequences into the final instruction format."""
        coordinate_sequence = [f"{x},{y},{z}" for x, y, z in processed_curves]  # Now correctly unpacks (x, y, z)
        return coordinate_sequence + ["GO"]  # Append 'GO' to indicate start

