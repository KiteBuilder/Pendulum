
#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "app.h"
#include "times.h"

//List of task IDs
typedef enum
{
    TASK_STROBE = 0,
    TASK_GYRO,
    TASK_PID_CONTROL,
    TASK_DISPLAY,
    TASK_POWER,
#ifdef DEBUG_ENABLED
    TASK_DEBUG,
#endif
    TASK_KEY,
    TASK_COUNT //amount of tasks
} taskId_e;

typedef struct
{
    const char *name; //name of the task, just applied for info
    void (*taskHandler)(timeUs_t currentTimeUs); //task handler routine
    timeDelta_t  taskPeriod; //desired period for task calls in microseconds
    bool taskEnabled; //true if the task enabled for the scheduling

    //Telemetry information to control the scheduler
    timeUs_t lastExecutedTime; // last time that task was executed
    timeDelta_t realPeriod; //measured period of time
    timeUs_t maxSpentTime; //maximal time in microseconds that was spent on the taskHandler
    timeUs_t totalSpentTime; //totally spent time, in microseconds
} task_t;

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

class TasksQueue {

public:
    TasksQueue(task_t* task_array, uint32_t size);
    ~TasksQueue();

    void reschedule(taskId_e, timeDelta_t period);
    void taskEnable(taskId_e id);
    void taskDisable(taskId_e id);
    void scheduler();

private:
    task_t *taskQueueArray;
    uint32_t taskQueueSize;
};

#endif /* SCHEDULER_H_ */
