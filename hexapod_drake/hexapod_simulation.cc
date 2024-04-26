#include <gflags/gflags.h>
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake{
namespace examples{
namespace hexapod_simulation{
namespace{

using drake::math::RigidTransformd;
using multibody::MultibodyPlant;
using Eigen::Translation3d;
using Eigen::VectorXd;
using drake::visualization::AddDefaultVisualization;


DEFINE_double(time_step, 1.0e-3,
        "Simulation time step used for integrator.");
DEFINE_double(simulation_time,10,
        "Desired duration of the simulation in seconds");
DEFINE_double(target_realtime_rate,1,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");
DEFINE_string(urdf,"","Name of urdf to load");
DEFINE_double(penetration_allowance, 1.0E-3, "Allowable penetration (meters).");
DEFINE_double(stiction_tolerance, 1.0E-3,
              "Allowable drift speed during stiction (m/s).");

DEFINE_double(
    mbp_discrete_update_period, 0.01,
    "The fixed-time step period (in seconds) of discrete updates for the "
    "multibody plant modeled as a discrete system. Strictly positive. "
    "Set to zero for a continuous plant model. When using TAMSI, a smaller "
    "time step of 1.0e-3 is recommended.");
DEFINE_string(contact_approximation, "sap",
              "Discrete contact approximation. Options are: 'tamsi', 'sap', "
              "'similar', 'lagged'");

static const char* const urdf_path = "package://drake/examples/hexapod_drake/urdf/hexapod_description.urdf";
// static const char* const urdf_path = "package://drake_models/jaco_description/urdf/j2s7s300_sphere_collision.urdf";

int DoMain(){
        DRAKE_DEMAND(FLAGS_simulation_time > 0);
        systems::DiagramBuilder<double> builder;

        auto [plant,scene_graph] = 
        multibody::AddMultibodyPlantSceneGraph(&builder,FLAGS_time_step);
        multibody::Parser(&plant).AddModelsFromUrl(urdf_path);

        //     Add model of the ground.
        const double static_friction = 1.0;
        const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
        plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                                geometry::HalfSpace(), "GroundVisualGeometry",
                                green);

        //     For a time-stepping model only static friction is used.
        const multibody::CoulombFriction<double> ground_friction(static_friction,
                                                                static_friction);
        plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(),
                                        geometry::HalfSpace(),
                                        "GroundCollisionGeometry", ground_friction);
        
        const drake::multibody::RigidBody<double>& body = 
                plant.GetBodyByName("body");
        // plant.AddJoint<multibody::WeldJoint>("weld_body",plant.world_body(), std::nullopt,
        //                 body, std::nullopt,
        //                 RigidTransformd(Vector3<double>{0,0,0.05}));
        // plant.WeldFrames(plant.world_frame(),
        //                 plant.GetFrameByName("body"));
        plant.Finalize();
        AddDefaultVisualization(&builder);
        plant.set_penetration_allowance(FLAGS_penetration_allowance);
        plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

        DRAKE_DEMAND(body.is_floating());

        // geometry::DrakeVisualizerd::AddToBuilder(&builder,scene_graph);
        std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
        std::unique_ptr<systems::Context<double>> diagram_context = 
                diagram->CreateDefaultContext();
        diagram->SetDefaultContext(diagram_context.get());

        // Create a context for this system:
        systems::Context<double>& plant_context =
                diagram->GetMutableSubsystemContext(plant, diagram_context.get());

        const VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs());
        plant.get_actuation_input_port().FixValue(&plant_context, tau);

        // Set the pelvis frame P initial pose.
        const Translation3d X_WP(0.0, 0.0, 0.005);
        plant.SetFreeBodyPoseInWorldFrame(&plant_context, body, X_WP);

        systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
        simulator.set_publish_every_time_step(true);
        simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
        simulator.Initialize();
        simulator.AdvanceTo(FLAGS_simulation_time);
        return 0;
}
}
}
}
}

int main(int argc, char* argv[]){
    gflags::ParseCommandLineFlags(&argc,&argv,true);
    return drake::examples::hexapod_simulation::DoMain();
}