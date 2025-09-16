#pragma once

#include "ConOps1/conops1.h"
#include "ConOps2/conops2.h"
#include "ConOps3/conops3.h"
#include <unordered_map>
using pybind11::len;
using pybind11::list;
using pybind11::array_t;
using pybind11::dict;

py::list conops1(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis, 
 const std::vector<std::tuple<int, std::vector<int>, std::vector<int>>> & orders, const bool & sparse, const bool & hub_to_hub, const int & order,
 const int & aggregated_demands_percent);

py::list conops2(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis, 
 const std::vector<std::tuple<int, std::vector<int>, std::vector<int>>> & orders, const bool & sparse, const bool & hub_to_hub, const int & order,
 const int & aggregated_demands_percent,
 int order_route_number, const py::list & order_dep,  const py::list & order_dest, const py::list & order_centerd, 
 const py::list & order_centera, const int & time_step, const int & window_size, array_t<int32_t> & updated_cells_type);

py::list conops3(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis, 
 const int & time_step, const int & window_size, const bool & sparse, const bool & hub_to_hub, const int & order,
 const int & aggregated_demands_percent);


py::list conops4(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis, 
 const std::vector<std::tuple<int, std::vector<int>, std::vector<int>>> & orders, const bool & sparse, const bool & hub_to_hub, const int & order,
 const int & aggregated_demands_percent,
 int order_route_number, const py::list & order_dep,  const py::list & order_dest, const py::list & order_centerd, 
 const py::list & order_centera, const int & time_step, const int & window_size, array_t<int32_t> & updated_cells_type);
