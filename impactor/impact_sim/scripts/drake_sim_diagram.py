from pydrake.all import (
    Diagram, DiagramBuilder, DrakeVisualizer,
    AddMultibodyPlantSceneGraph
)

class DrakeSimDiagram(Diagram):
    def __init__(self, config):
        Diagram.__init__(self)

        dt = config["mbp_dt"]
        self._builder = DiagramBuilder()
        self._mbp, self._sg = AddMultibodyPlantSceneGraph(self._builder, time_step=dt)
        self._model_ids = dict()
        self._body_ids = dict()

        self._finalize_functions = []
        self._finalized = False

    # === Property accessors ========================================
    @property
    def mbp(self):
        return self._mbp
    
    @property
    def sg(self):
        return self._sg

    @property
    def builder(self):
        return self._builder

    @property
    def model_ids(self):
        return self._model_ids
    
    @property
    def finalize_functions(self):
        return self._finalize_functions

    # === Add visualizer ============================================
    def connect_to_drake_visualizer(self):
        self._drake_viz = DrakeVisualizer.AddToBuilder(
            builder = self._builder, scene_graph = self._sg)
        return self._drake_viz

    # === Finalize the complete diagram =============================
    def finalize(self):
        if self._finalized:
            return

        self._mbp.Finalize()
        for func in self._finalize_functions:
            func()

        self._builder.BuildInto(self)
        self._finalized = True

    def is_finalized(self):
        return self._finalized