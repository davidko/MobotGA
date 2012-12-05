#include <iostream>
#include <algorithm>
#include <ode/ode.h>
#include "pandaFramework.h"
#include "pandaSystem.h"
#include "cIntervalManager.h"
#include "mobot_model.h"
#include "texturePool.h"
#include "pointLight.h"
#include "ambientLight.h"
#include "main.h"
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
uint8_t a[4][5];
uint8_t b[4][5];
bool gEnableGraphics = true;
 
// Create an accumulator to track the time since the sim
// has been running
float deltaTimeAccumulator = 0.0f;
 
// This stepSize makes the simulation run at 90 frames per second
double stepSize = 1.0 / 90.0;
//float stepSize = 0.05;
WindowFramework *window;
PandaFramework framework;
 
 
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
    //contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSlip1 | dContactSlip2;
    contact[i].surface.mode = dContactBounce | dContactSoftCFM ;
    //contact[i].surface.mu = dInfinity;
    contact[i].surface.mu = 0.01;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
    contact[i].surface.slip1 = 0.5;
    contact[i].surface.slip2 = 0.5;
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
  /* Delete old fourier coefficients file */
  unlink("/tmp/fourier_coefs.txt");

  /* Process command line arguments */
  int i;
  FILE *coefs = NULL;
  for(i = 1; i < argc; i++) {
    if(
        !strcmp(argv[i], "--help") ||
        !strcmp(argv[i], "-h")
        ) {
      printf("Available options:\n"
             "  --disable-graphics: Run w/out opening visualization window\n"
             "  --load-coefs <filename> : Load coefficients file\n");
      exit(0);
    }
    if(!strcmp(argv[i], "--disable-graphics")) {
      gEnableGraphics=false;
    }
    if(!strcmp(argv[i], "--load-coefs")) {
      i++;
      if(i >= argc) {
        fprintf(stderr, "Error: --load-coefs expects a filename\n");
        return 0;
      }
      coefs = fopen(argv[i], "r");
      if(coefs == NULL) {
        fprintf(stderr, "Error opening file: %s\n", argv[i]);
        return 0;
      }
    }
  }

  if(gEnableGraphics) {
    stepSize = 1.0f / 30.0;
    initGraphics(argc, argv);
  } else {
    stepSize = 1.0f / 30.0;
  }

#if 0
  // Add our task.
  taskMgr->add(new GenericAsyncTask("Spins the camera",
    &SpinCameraTask, (void*) NULL));
#endif

  simulation(coefs);
  int initTime = time(NULL);

  if(gEnableGraphics) {
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

    closeGraphics(); 
  } else {
    double time = 0;
    while(1) {
      printf("%lf ", time);
      dSpaceCollide (space,0,&nearCallback);
      dWorldStep(world, stepSize);
      chain->step(time);
      dJointGroupEmpty(contactgroup);
      time += stepSize;
      if(time > 30) {break;}
    }
  }
  return (0);
}

void initGraphics(int argc, char *argv[])
{
  // Open a new window framework and set the title
  framework.open_framework(argc, argv);
  framework.set_window_title("My Panda3D Window");
 
  // Open the window
  window = framework.open_window();
  window->set_texture(false);
  //window->setup_trackball();
  camera = window->get_camera_group(); // Get the camera and store it

  /* Set up lighting */
  PointLight* p_light;
  p_light = new PointLight("sun");
  p_light->set_color(LVecBase4f(0.9, 0.9, 0.9, 1));
  NodePath plightSun_p = window->get_render().attach_new_node(p_light);
  plightSun_p.set_pos(15, 15, 5);
  window->get_render().set_light(plightSun_p);

  AmbientLight* alight = new AmbientLight("my alight");
  alight->set_color(LVecBase4f(0.8, 0.8, 0.8, 0.5));
  NodePath alnp = window->get_render().attach_new_node(alight);
  window->get_render().set_light(alnp);
}

void closeGraphics(void)
{
  framework.close_framework();
}

void simulation(FILE* coefs){
  srand(time(NULL));
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
  dCreatePlane (space,0,0,1,-0.5);
  dSpaceCollide (space,0,&nearCallback);

  if(gEnableGraphics) {
    chain = new MobotChain(window, &framework, world, space, 3, coefs);
  } else {
    chain = new MobotChain(NULL, NULL, world, space, 3, coefs);
  }
  //chain->mobot(0)->moveTo(0, DEG2RAD(90), 0, 0);
  //chain->mobot(2)->moveTo(0, 0, DEG2RAD(90), 0);

  PT(GenericAsyncTask) simulationTaskObject =
    new GenericAsyncTask("startup task", &simulationTask, (void*) NULL);
  //simulationTaskObject->set_delay(4);
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
    if(globalClock->get_real_time() > WAIT_TIME) {
      chain->step();
    }
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
  angledegrees = 120.0;
  double angleradians = angledegrees * (3.14 / 180.0);
  camera.set_pos(-2*sin(angleradians),-2*cos(angleradians),1);
  //camera.set_pos(LVector3f(pos[0], pos[1], pos[2]) + LVector3f(1, 1, 1));
  camera.look_at(LVector3f(pos[0], pos[1], pos[2]));

  return AsyncTask::DS_cont;
}
