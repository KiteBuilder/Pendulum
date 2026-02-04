#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "Scheduler.h"

using namespace std;

/**
  * @brief Constructor
  * @param None
  * @retval None
  */
TasksQueue::TasksQueue(task_t* task_array, uint32_t size): taskQueueArray(task_array), taskQueueSize(size)
{

}

/**
  * @brief Destructor
  * @param None
  * @retval None
  */
TasksQueue::~TasksQueue()
{

}

/**
  * @brief To change desired period for the selected task
  * @param id: task index in the task array
  *        period: new desired period in microseconds
  * @retval
  */
void TasksQueue::reschedule(taskId_e id, timeDelta_t period)
{
    if (id < taskQueueSize)
    {
        task_t *task = &taskQueueArray[id];
        task->taskPeriod = period;
    }
}

/**
  * @brief Enable selected task
  * @param id: task index in the task array
  * @retval None
  */
void TasksQueue::taskEnable(taskId_e id)
{
    if (id < taskQueueSize)
    {
        task_t *task = &taskQueueArray[id];
        task->taskEnabled = true;
    }
}

/**
  * @brief Disable selected task
  * @param id: task index in the task array
  * @retval None
  */
void TasksQueue::taskDisable(taskId_e id)
{
    if (id < taskQueueSize)
    {
        task_t *task = &taskQueueArray[id];
        task->taskEnabled = false;
    }
}

/**
  * @brief Tasks scheduler - going through all task array and only one enabled task, with elapsed time
  *        period, should be selected. This routine should be located in the main loop with no HAL_Delay
  *        or like that restrictions
  * @param
  * @retval
  */
void TasksQueue::scheduler()
{
    task_t *selectedTask = NULL;

    timeUs_t currentTime = micros();

    for (uint32_t i = 0; i < taskQueueSize; i++)
    {
        task_t *task = &taskQueueArray[i];
        if ((timeDelta_t)(currentTime - task->lastExecutedTime) > task->taskPeriod && task->taskEnabled)
        {
            selectedTask = task;
            break;
        }
    }

    if (selectedTask != NULL)
    {
        currentTime = micros();
        selectedTask->realPeriod = (timeDelta_t)(currentTime - selectedTask->lastExecutedTime);
        selectedTask->lastExecutedTime = currentTime;
        selectedTask->taskHandler(currentTime); //Execute task
        timeUs_t executionTime = micros() - currentTime;
        selectedTask->totalSpentTime += executionTime;
        selectedTask->maxSpentTime = selectedTask->maxSpentTime > executionTime ? selectedTask->maxSpentTime : executionTime;
    }
}
