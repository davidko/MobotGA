#include <iostream>
#include <algorithm>
#include "pandaFramework.h"
#include "pandaSystem.h"
 
#include "genericAsyncTask.h"
#include "asyncTaskManager.h"
 
#include "cIntervalManager.h"
#include "cLerpNodePathInterval.h"
#include "cMetaInterval.h"

#include "odeWorld.h"
#include "odeBody.h"
#include "odeMass.h"
#include "odeSimpleSpace.h"
#include "odeBoxGeom.h"
#include "cardMaker.h"
#include "odePlaneGeom.h"

#include "mobot_model.h"

using namespace std;

// Global stuff
 
OdeBody *body;
OdeWorld world;
NodePath sphere;
NodePath cyl;
OdeBody *cylBody;
OdeSimpleSpace* space;
OdeJointGroup* contactgroup;
MobotModel* mobot;
PT(ClockObject) globalClock = ClockObject::get_global_clock();
 
// Create an accumulator to track the time since the sim
// has been running
float deltaTimeAccumulator = 0.0f;
 
// This stepSize makes the simulation run at 90 frames per second
float stepSize = 1.0f / 90.0f;
//float stepSize = 0.05;
WindowFramework *window;
PandaFramework framework;
 
AsyncTask::DoneStatus simulationTask (GenericAsyncTask* task, void* data);
void simulation();
 
 
PT(AsyncTaskManager) taskMgr = AsyncTaskManager::get_global_ptr(); 
NodePath camera;
 
// Task to move the camera
AsyncTask::DoneStatus SpinCameraTask(GenericAsyncTask* task, void* data) {
  double time = globalClock->get_real_time();
  double angledegrees = time * 6.0;
  double angleradians = angledegrees * (3.14 / 180.0);
  camera.set_pos(20*sin(angleradians),-20.0*cos(angleradians),3);
  camera.set_hpr(angledegrees, 0, 0);
 
  return AsyncTask::DS_cont;
}
 
int main(int argc, char *argv[]) {
  // Open a new window framework and set the title
  framework.open_framework(argc, argv);
  framework.set_window_title("My Panda3D Window");
 
  // Open the window
  window = framework.open_window();
  //window->setup_trackball();
  camera = window->get_camera_group(); // Get the camera and store it


#if 0 
  // Load the environment model
  NodePath environ = window->load_model(framework.get_models(),
    "models/environment");
  environ.reparent_to(window->get_render());
  environ.set_scale(0.25 , 0.25, 0.25);
  environ.set_pos(-8, 42, 0);
#endif

#if 0
  // Add our task.
  taskMgr->add(new GenericAsyncTask("Spins the camera",
    &SpinCameraTask, (void*) NULL));
#endif

  simulation();

  // Set the camera position
  camera.set_pos (1, 1, 1);
  camera.look_at (0, 0, 0);
 
  // This is a simpler way to do stuff every frame,
  // if you're too lazy to create a task.
  Thread *current_thread = Thread::get_current_thread();
  while(framework.do_frame(current_thread)) {
    // Step the interval manager
    CIntervalManager::get_global_ptr()->step();
  }
 
  framework.close_framework();
  return (0);
}

void simulation(){
  // Load the cube where the ball will fall from
#if 0
  NodePath cube = window->load_model(framework.get_models(), "models/box");
  cube.reparent_to(window->get_render());
  cube.set_scale(0.25, 0.25, 0.25);
  cube.set_pos(0, 0, 0);
#endif
 
  // Setup our physics world and the body
  world.set_gravity( 0 , 0, -.81 );
  world.init_surface_table(1);
  world.set_surface_entry(
      0,  // id 1
      0,  // id 2
      150, // mu
      0.0, // bounce
      9.1, //bounce_vel
      0.9, //soft_erp
      0.00001,  //soft_cfm
      0.0,  //slip
      0.02); //dampen
  world.set_contact_surface_layer(0.001);

  // Setup our contact space, etc.
  space = new OdeSimpleSpace();
  space->set_auto_collide_world(world);
  contactgroup = new OdeJointGroup();
  space->set_auto_collide_joint_group(*contactgroup);

  mobot = new MobotModel(window, &framework, &world, space);
  /* Create the body */
#if 0
  body = new OdeBody(world);
  OdeMass M = OdeMass();
  M.set_box(50, 1.0, 1.0, 1.0);
  body->set_mass(M);
  body->set_position(sphere.get_pos(window->get_render()));
  body->set_quaternion(sphere.get_quat(window->get_render()));
  OdeBoxGeom* boxGeom = new OdeBoxGeom(*space, (dReal)1.0, (dReal)1.0, (dReal)1.0);
  boxGeom->set_collide_bits(0x02);
  boxGeom->set_category_bits(0x01);
  boxGeom->set_body(*body);
#endif
  LQuaternionf q;
  q.set_from_axis_angle(0, LVector3f(1, 0, 0));
  mobot->build_mobot(0, 0, 0.3, q);
  //mobot->build_faceplate1(0, 0, 0.3, q);
  //mobot->build_body1(0, 0, .3, q );
  //mobot->build_center(0, 0, 4, sphere.get_quat(window->get_render()));

  /* Create ground plane */
  CardMaker* cm = new CardMaker("ground");
  cm->set_frame(-20, 20, -20, 20);
  //cm->set_frame(-1, 1, -1, 1);
  NodePath ground = window->get_render().attach_new_node(cm->generate());
  ground.set_pos(0,0,0); ground.look_at(0, 0, -1);
  OdePlaneGeom* groundGeom = new OdePlaneGeom(*space, (dReal)0, (dReal)0, (dReal)1, (dReal)0);
  groundGeom->set_collide_bits(0xFF);
  groundGeom->set_category_bits(0x20);

 
  PT(GenericAsyncTask) simulationTaskObject =
    new GenericAsyncTask("startup task", &simulationTask, (void*) NULL);
  simulationTaskObject->set_delay(2);
  taskMgr->add(simulationTaskObject);
}
 
// The task for our simulation
AsyncTask::DoneStatus simulationTask (GenericAsyncTask* task, void* data) {
  space->auto_collide();
  // Set the force on the body to push it off the ridge
  //body->set_force(0, min(pow(task->get_elapsed_time(),4.0) * 500000 - 500000, 0.0), 0);
  // Add the deltaTime for the task to the accumulator
  deltaTimeAccumulator += globalClock->get_dt();
  while (deltaTimeAccumulator > stepSize ) {
    // Remove a stepSize from the accumulator until
    // the accumulated time is less than the stepsize
    deltaTimeAccumulator -= stepSize;
    // Step the simulation
    world.step(stepSize);
  }
  // set the new positions
  mobot->update();
  camera.set_pos(mobot->get_position(0) + LVector3f(1, 1, 1));
  camera.look_at(mobot->get_position(0));

  contactgroup->empty();
  return AsyncTask::DS_cont;
}
