#pragma once
#include <iostream>
#include <vector>

typedef struct DroneLimits {
  // 最大速度
  double max_fly_speed_v;
  double max_fly_speed_h;
  // 最大飞行加速度
  double max_fly_acc_v;
  double max_fly_acc_h;
  // 最大载重
  double max_weight;
  // 最大货物数
  double max_cargo_slots;
  double min_fly_height;
  double max_fly_height;
  // 最大飞行时间
  double max_flight_seconds;
} DroneLimits;