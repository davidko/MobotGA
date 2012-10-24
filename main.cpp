#include <iostream>
#include <algorithm>
#include <ode/ode.h>
#include "pandaFramework.h"
#include "pandaSystem.h"
#include "cIntervalManager.h"
#include "mobot_model.h"
using namespace std;

#define MAX_CONTACTS 8          // maximum number of contact points per body
// Global stuff
 
dWorldID world;
dSpaceID space;
NodePath sphere;
NodePath cyl;
dBodyID body;
    dGeomID geom;
dJointGroupID contactgroup;
MobotModel* mobot, *mobot2;
MobotChain* chain;
PT(ClockObject) globalClock = ClockObject::get_global_clock();
int init = 1;
 
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
 
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
        sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
    }
  }
}

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

  // create world
  dInitODE();
  dAllocateODEDataForThread(dAllocateMaskAll);
  world = dWorldCreate();
  //space = dHashSpaceCreate (0);
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
 
  // Setup our physics world and the body
  dWorldSetGravity(world, 0 , 0, -9.81 );
  dWorldSetCFM (world,1e-5);
  dWorldSetAutoDisableFlag (world,1);

#if 1

  dWorldSetAutoDisableAverageSamplesCount( world, 10 );

#endif

  dWorldSetLinearDamping(world, 0.00001);
  dWorldSetAngularDamping(world, 0.005);
  dWorldSetMaxAngularSpeed(world, 200);

  dWorldSetContactMaxCorrectingVel (world,0.1);
  dWorldSetContactSurfaceLayer (world,0.001);
  dCreatePlane (space,0,0,1,0);
  dSpaceCollide (space,0,&nearCallback);

  //mobot = new MobotModel(window, &framework, world, space);
  //mobot2 = new MobotModel(window, &framework, world, space);
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
#if 0
  LQuaternionf q;
  q.set_from_axis_angle(9, LVector3f(1, 0, 0));
  //mobot->build_mobot(0, 0, 0.3, q);
  //mobot2->build_mobot(0.3, 0, 0.3, q);
  //mobot->build_mobot_chain_head(0, 0, 0.3);
  mobot2->build_mobot_chain_tail(
      0.0, 
      FACEPLATE_Y/2.0 + BODY_Y*2.0 + FACEPLATE_COMPOUND_Y/2.0, 
      0.3);
  mobot->attach_mobot(mobot2);
  //mobot->build_faceplate1(0, 0, 0.3, q);
  //mobot->build_body1(0, 0, .3, q );
  //mobot->build_center(0, 0, 4, sphere.get_quat(window->get_render()));
#endif
  chain = new MobotChain(window, &framework, world, space, 4);

  PT(GenericAsyncTask) simulationTaskObject =
    new GenericAsyncTask("startup task", &simulationTask, (void*) NULL);
  simulationTaskObject->set_delay(2);
  taskMgr->add(simulationTaskObject);
}
 
// The task for our simulation
AsyncTask::DoneStatus simulationTask (GenericAsyncTask* task, void* data) {
  if(init) {
    dAllocateODEDataForThread(dAllocateMaskAll);
    init = 0;
  }
  // Set the force on the body to push it off the ridge
  //body->set_force(0, min(pow(task->get_elapsed_time(),4.0) * 500000 - 500000, 0.0), 0);
  // Add the deltaTime for the task to the accumulator
  deltaTimeAccumulator += globalClock->get_dt();
  while (deltaTimeAccumulator > stepSize ) {
    // Remove a stepSize from the accumulator until
    // the accumulated time is less than the stepsize
    deltaTimeAccumulator -= stepSize;
    // Step the simulation
    dSpaceCollide (space,0,&nearCallback);
    dWorldStep(world, stepSize);
    chain->step();
    //mobot->step();
    //mobot2->step();
    //world.step(stepSize);
    dJointGroupEmpty(contactgroup);
  }
  // set the new positions
  chain->update();
  //mobot->update();
  //mobot2->update();
  mobot = chain->mobot(0);
  const dReal *pos = mobot->get_position(0);
  double time = globalClock->get_real_time();
  double angledegrees = time * 30.0;
  double angleradians = angledegrees * (3.14 / 180.0);
  camera.set_pos(sin(angleradians),cos(angleradians),1);
  //camera.set_pos(LVector3f(pos[0], pos[1], pos[2]) + LVector3f(1, 1, 1));
  camera.look_at(LVector3f(pos[0], pos[1], pos[2]));

  return AsyncTask::DS_cont;
}
