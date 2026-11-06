// Selects the generated robot model compiled into the controller core.
// Default: robot_model.h (generated from CAD/scene.xml). Build with
//   -DWBC_ROBOT_MODEL='"models/<name>/robot_model.h"'
// for another robot (e.g. models/bd, generated from CAD_bd/scene.xml by
// tools/gen_robot_model.py --xml CAD_bd/scene.xml --out src/wbc/models/bd).
#pragma once
#ifdef WBC_ROBOT_MODEL
#include WBC_ROBOT_MODEL
#else
#include "robot_model.h"
#endif
