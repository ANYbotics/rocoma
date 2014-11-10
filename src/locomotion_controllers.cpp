
#include <locomotion_controller/ControllerManager.hpp>

#ifdef USE_TASK_LOCODEMO
#include "LocoDemo_Task.hpp"
#endif

namespace locomotion_controller {

void add_locomotion_controllers(locomotion_controller::ControllerManager* manager, model::Model* model) {

#ifdef USE_TASK_LOCODEMO
   manager->addController(new robotTask::LocoDemo(model->getRobotModel(), model->getTerrainModel()));
#endif


}

}
