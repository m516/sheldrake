#include <memory>
#include <iostream>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/visualization/visualization_config_functions.h"

using namespace drake;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::JointActuator;
using multibody::RevoluteJoint;
using multibody::PrismaticJoint;
using systems::Context;
using systems::InputPort;
using Eigen::Vector2d;






namespace cartpole {





const std::string cart_pole_sdf = "                                             \
<?xml version=\"1.0\"?>                                                          \
<sdf version=\"1.7\">                                                            \
  <model name=\"CartPole\">                                                      \
    <!-- This sdf file produces a model with the default parameters as           \
         documented in cart_pole_params_named_vector.yaml.                       \
         They MUST be kept in sync. -->                                          \
    <link name=\"Cart\">                                                         \
      <inertial>                                                                 \
        <mass>10.0</mass>                                                        \
        <!-- For this model case, with the cart not having any rotational        \
             degrees of freedom, the values of the inertia matrix do not         \
             participate in the model. Therefore we just set them to zero        \
             (or near to zero since sdformat does not allow exact zeroes         \
             for inertia values). -->                                            \
        <inertia>                                                                \
          <ixx>1.0e-20</ixx>                                                     \
          <iyy>1.0e-20</iyy>                                                     \
          <izz>1.0e-20</izz>                                                     \
          <ixy>0</ixy>                                                           \
          <ixz>0</ixz>                                                           \
          <iyz>0</iyz>                                                           \
        </inertia>                                                               \
      </inertial>                                                                \
      <visual name=\"cart_visual\">                                              \
        <geometry>                                                               \
          <box>                                                                  \
            <size>0.24 0.12 0.12</size>                                          \
          </box>                                                                 \
        </geometry>                                                              \
      </visual>                                                                  \
    </link>                                                                      \
    <link name=\"Pole\">                                                         \
      <!-- The pole is modeled as a point mass at the end of a pole. -->         \
      <!-- The length of the pole is 0.5 meters. -->                             \
      <pose>0 0 -0.5 0 0 0</pose>                                                \
      <inertial>                                                                 \
        <mass>1.0</mass>                                                         \
        <!-- A point mass has zero rotational inertia.                           \
             We must specify small values since otherwise sdformat throws an     \
             exception. -->                                                      \
        <inertia>                                                                \
          <ixx>1.0e-20</ixx>                                                     \
          <iyy>1.0e-20</iyy>                                                     \
          <izz>1.0e-20</izz>                                                     \
          <ixy>0</ixy>                                                           \
          <ixz>0</ixz>                                                           \
          <iyz>0</iyz>                                                           \
        </inertia>                                                               \
      </inertial>                                                                \
      <visual name=\"pole_point_mass\">                                          \
        <geometry>                                                               \
          <sphere>                                                               \
            <radius>0.05</radius>                                                \
          </sphere>                                                              \
        </geometry>                                                              \
      </visual>                                                                  \
      <visual name=\"pole_rod\">                                                 \
        <pose>0 0 0.25 0 0 0</pose>                                              \
        <geometry>                                                               \
          <cylinder>                                                             \
            <radius>0.025</radius>                                               \
            <length>0.5</length>                                                 \
          </cylinder>                                                            \
        </geometry>                                                              \
      </visual>                                                                  \
    </link>                                                                      \
    <joint name=\"CartSlider\" type=\"prismatic\">                               \
      <parent>world</parent>                                                     \
      <child>Cart</child>                                                        \
      <axis>                                                                     \
        <xyz>1.0 0.0 0.0</xyz>                                                   \
      </axis>                                                                    \
    </joint>                                                                     \
    <joint name=\"PolePin\" type=\"revolute\">                                   \
      <!-- Pose of the joint frame in the pole's frame (located at the point     \
           mass) -->                                                             \
      <pose>0 0 0.5 0 0 0</pose>                                                 \
      <parent>Cart</parent>                                                      \
      <child>Pole</child>                                                        \
      <axis>                                                                     \
        <xyz>0.0 -1.0 0.0</xyz>                                                  \
        <limit>                                                                  \
          <!-- The pole pin joint is not actuated. -->                           \
          <effort>0</effort>                                                     \
        </limit>                                                                 \
      </axis>                                                                    \
    </joint>                                                                     \
  </model>                                                                       \
</sdf>                                                                           \
";




DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 3.0,
              "Desired duration of the simulation in seconds.");

DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");

// This helper method makes an LQR controller to balance cart_pole model
// specified in the SDF file `file_name`.
std::unique_ptr<systems::AffineSystem<double>> MakeBalancingLQRController() {
  // LinearQuadraticRegulator() below requires the controller's model of the
  // plant to only have a single input port corresponding to the actuation.
  // Therefore we create a new model that meets this requirement. (a model
  // created along with a SceneGraph for simulation would also have input ports
  // to interact with that SceneGraph).
  MultibodyPlant<double> cart_pole(0.0);
  Parser parser(&cart_pole);
  parser.AddModelsFromString(cart_pole_sdf, "sdf");
  // We are done defining the model.
  cart_pole.Finalize();

  const PrismaticJoint<double>& cart_slider =
      cart_pole.GetJointByName<PrismaticJoint>("CartSlider");
  const RevoluteJoint<double>& pole_pin =
      cart_pole.GetJointByName<RevoluteJoint>("PolePin");
  std::unique_ptr<Context<double>> context = cart_pole.CreateDefaultContext();

  // Set nominal actuation torque to zero.
  const InputPort<double>& actuation_port = cart_pole.get_actuation_input_port();
  actuation_port.FixValue(context.get(), 0.0);
  cart_pole.get_applied_generalized_force_input_port().FixValue(
      context.get(), Vector2d::Constant(0.0));


  cart_slider.set_translation(context.get(), 0.0);
  cart_slider.set_translation_rate(context.get(), 0.0);
  pole_pin.set_angle(context.get(), M_PI);
  pole_pin.set_angular_rate(context.get(), 0.0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Vector1d R = Vector1d::Constant(1);

  return systems::controllers::LinearQuadraticRegulator(
      cart_pole, *context, Q, R,
      Eigen::Matrix<double, 0, 0>::Zero() /* No cross state/control costs */,
      actuation_port.get_index());
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  const double time_step = FLAGS_time_stepping ? 1.0e-3 : 0.0;
  auto [cart_pole, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, time_step);

  // Make and add the cart_pole model.
  Parser parser(&cart_pole);
  parser.AddModelsFromString(cart_pole_sdf, "sdf");

  // We are done defining the model.
  cart_pole.Finalize();

  DRAKE_DEMAND(cart_pole.num_actuators() == 1);
  DRAKE_DEMAND(cart_pole.num_actuated_dofs() == 1);

  PrismaticJoint<double>& cart_slider =
      cart_pole.GetMutableJointByName<PrismaticJoint>("CartSlider");
  RevoluteJoint<double>& pole_pin =
      cart_pole.GetMutableJointByName<RevoluteJoint>("PolePin");

  // Drake's parser will default the name of the actuator to match the name of
  // the joint it actuates.
  const JointActuator<double>& actuator =
      cart_pole.GetJointActuatorByName("CartSlider");
  DRAKE_DEMAND(actuator.joint().name() == "CartSlider");

  // For this example the controller's model of the plant exactly matches the
  // plant to be controlled (in reality there would always be a mismatch).
  auto controller = builder.AddSystem(
      MakeBalancingLQRController());
  controller->set_name("controller");
  builder.Connect(cart_pole.get_state_output_port(),
                  controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  cart_pole.get_actuation_input_port());

  // Get the controller design
  std::cout << "A: " << controller->A() << std::endl;
  std::cout << "B: " << controller->B() << std::endl;
  std::cout << "C: " << controller->C() << std::endl;
  std::cout << "D: " << controller->D() << std::endl;

  visualization::AddDefaultVisualization(&builder);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

  RandomGenerator generator;

  // Setup distribution for random initial conditions.
  std::normal_distribution<symbolic::Expression> gaussian;
  cart_slider.set_random_translation_distribution(0);
  pole_pin.set_random_angle_distribution(M_PI + 0.2*gaussian(generator));

  for (int i = 0; i < 50; i++) {
    simulator.get_mutable_context().SetTime(0.0);
    simulator.get_system().SetRandomContext(&simulator.get_mutable_context(),
                                            &generator);

    simulator.Initialize();
    simulator.AdvanceTo(FLAGS_simulation_time);
  }

  return 0;
}

}  // namespace cartpole

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple cart pole demo using Drake's MultibodyPlant with "
      "LQR stabilization. "
      "Launch meldis before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return cartpole::do_main();
}