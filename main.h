#ifndef _MAIN_H_
#define _MAIN_H_

static void nearCallback (void *data, dGeomID o1, dGeomID o2);
void initGraphics(int argc, char *argv[]);
void closeGraphics(void);
void simulation();
AsyncTask::DoneStatus simulationTask (GenericAsyncTask* task, void* data);

#endif
