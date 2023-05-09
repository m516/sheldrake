// This is an official Drake example, written by the Drake community.
/*
All components of Drake are licensed under the BSD 3-Clause License
shown below. Where noted in the source code, some portions may 
be subject to other permissive, non-viral licenses.

Copyright 2012-2022 Robot Locomotion Group @ CSAIL
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
*/

#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"





const std::string cart_pole_udrf = "                                             \
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

using namespace drake;
using geometry::SceneGraph;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");





int do_main() {
  systems::DiagramBuilder<double> builder;

  // Make and add the cart_pole model.
  auto [cart_pole, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);
  const std::string sdf_url =
      "package://drake/examples/multibody/cart_pole/cart_pole.sdf";
  Parser(&cart_pole, &scene_graph).AddModelsFromString(cart_pole_udrf, "sdf");

  // Now the model is complete.
  cart_pole.Finalize();

  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& cart_pole_context =
      diagram->GetMutableSubsystemContext(cart_pole, diagram_context.get());

  // There is no input actuation in this example for the passive dynamics.
  cart_pole.get_actuation_input_port().FixValue(&cart_pole_context, 0.1);

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& cart_slider =
      cart_pole.GetJointByName<PrismaticJoint>("CartSlider");
  const RevoluteJoint<double>& pole_pin =
      cart_pole.GetJointByName<RevoluteJoint>("PolePin");

  // Set initial state.
  cart_slider.set_translation(&cart_pole_context, 0.0);
  pole_pin.set_angle(&cart_pole_context, 2.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}




int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple cart pole demo using Drake's MultibodyPlant,"
      "with SceneGraph visualization. "
      "Launch meldis before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return do_main();
}