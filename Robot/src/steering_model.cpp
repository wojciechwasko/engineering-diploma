#include "steering_model.hpp"

#include <cmath>
#include <iostream>

using SeekurJrRC::Core::SteeringModelBase;

const float SteeringModelBase::c_arr_phi[9] =   { -90,    0,   90, -90,   0,  90, -90,   0,   90};
const float SteeringModelBase::c_arr_theta[9] = { -90,  -90,  -90,   0,   0,   0,  90,  90,   90};
const float SteeringModelBase::c_arr_z_a[9] =   {-0.5, -1.0, -0.5, 0.0, 0.0, 0.0, 0.5, 1.0,  0.5};
const float SteeringModelBase::c_arr_z_b[9] =   {-0.5,    0,  0.5, 0.0, 0.0, 0.0, 0.5, 0.0, -0.5};

using SeekurJrRC::Core::BilinearNoFiltModel;
using SeekurJrRC::Core::BilinearSimpleFiltModel;
using SeekurJrRC::Core::BilinearExpFiltModel;

using SeekurJrRC::Core::Shepard1_5NoFiltModel;
using SeekurJrRC::Core::Shepard1_5SimpleFiltModel;
using SeekurJrRC::Core::Shepard1_5ExpFiltModel;

using SeekurJrRC::Core::Shepard4_5NoFiltModel;
using SeekurJrRC::Core::Shepard4_5SimpleFiltModel;
using SeekurJrRC::Core::Shepard4_5ExpFiltModel;

using SeekurJrRC::Core::Genetic1NoFiltModel;
using SeekurJrRC::Core::Genetic1SimpleFiltModel;
using SeekurJrRC::Core::Genetic1ExpFiltModel;

using SeekurJrRC::Core::Genetic2NoFiltModel;
using SeekurJrRC::Core::Genetic2SimpleFiltModel;
using SeekurJrRC::Core::Genetic2ExpFiltModel;


// instancja historii - żeby kompilator się nie burzył
std::deque<SeekurJrRC::Core::acc_tuple> SteeringModelBase::history_;

std::pair<float, float> BilinearNoFiltModel::getSpeedValues()
{
//   std::cout << "phi: " << getPhiDeg(current_acc_) << "theta: " << getThetaDeg(current_acc_) << std::endl;
  return speedValuesFromAB(bilinear(current_acc_));
}

std::pair<float, float> BilinearSimpleFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredSimple();
  return speedValuesFromAB(bilinear(acc));
}

std::pair<float, float> BilinearExpFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredExponential();
  return speedValuesFromAB(bilinear(acc));
}

std::pair<float, float> Shepard1_5NoFiltModel::getSpeedValues()
{
  return speedValuesFromAB(shepard(current_acc_, 1.5));
}

std::pair<float, float> Shepard1_5SimpleFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredSimple();
  return speedValuesFromAB(shepard(acc, 1.5));
}

std::pair<float, float> Shepard1_5ExpFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredExponential();
  return speedValuesFromAB(shepard(acc, 1.5));
}

std::pair<float, float> Shepard4_5NoFiltModel::getSpeedValues()
{
  return speedValuesFromAB(shepard(current_acc_, 4.5));
}

std::pair<float, float> Shepard4_5SimpleFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredSimple();
  return speedValuesFromAB(shepard(acc, 4.5));
}

std::pair<float, float> Shepard4_5ExpFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredExponential();
  return speedValuesFromAB(shepard(acc, 4.5));
}

std::pair<float, float> Genetic1NoFiltModel::getSpeedValues()
{
  return speedValuesFromAB(genetic1(current_acc_));
}

std::pair<float, float> Genetic1SimpleFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredSimple();
  return speedValuesFromAB(genetic1(acc));
}

std::pair<float, float> Genetic1ExpFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredExponential();
  return speedValuesFromAB(genetic1(acc));
}

std::pair<float, float> Genetic2NoFiltModel::getSpeedValues()
{
  return speedValuesFromAB(genetic2(current_acc_));
}

std::pair<float, float> Genetic2SimpleFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredSimple();
  return speedValuesFromAB(genetic2(acc));
}

std::pair<float, float> Genetic2ExpFiltModel::getSpeedValues()
{
  acc_tuple acc = getCurrentValueFilteredExponential();
  return speedValuesFromAB(genetic2(acc));
}

boost::shared_ptr<SteeringModelBase> SeekurJrRC::Core::getSteeringModel(uint8_t modelCode, const float& x, const float& y, const float& z)
{
    
  if (modelCode == BILINEAR_NO_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new BilinearNoFiltModel(x,y,z));
  if (modelCode == BILINEAR_SIMPLE_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new BilinearSimpleFiltModel(x,y,z));
  if (modelCode == BILINEAR_EXP_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new BilinearExpFiltModel(x,y,z));
  if (modelCode == SHEPARD_1_5_NO_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Shepard1_5NoFiltModel(x,y,z));
  if (modelCode == SHEPARD_1_5_SIMPLE_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Shepard1_5SimpleFiltModel(x,y,z));
  if (modelCode == SHEPARD_1_5_EXP_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Shepard1_5ExpFiltModel(x,y,z));
  if (modelCode == SHEPARD_4_5_NO_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Shepard4_5NoFiltModel(x,y,z));
  if (modelCode == SHEPARD_4_5_SIMPLE_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Shepard4_5SimpleFiltModel(x,y,z));
  if (modelCode == SHEPARD_4_5_EXP_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Shepard4_5ExpFiltModel(x,y,z));
//   if (modelCode == GENETIC_1_NO_FILT_MODEL_CODE)
//     return boost::shared_ptr<SteeringModelBase>(new Genetic1NoFiltModel(x,y,z));
//   if (modelCode == GENETIC_1_SIMPLE_FILT_MODEL_CODE)
//     return boost::shared_ptr<SteeringModelBase>(new Genetic1SimpleFiltModel(x,y,z));
//   if (modelCode == GENETIC_1_EXP_FILT_MODEL_CODE)
//     return boost::shared_ptr<SteeringModelBase>(new Genetic1ExpFiltModel(x,y,z));
  if (modelCode == GENETIC_2_NO_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Genetic2NoFiltModel(x,y,z));
  if (modelCode == GENETIC_2_SIMPLE_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Genetic2SimpleFiltModel(x,y,z));
  if (modelCode == GENETIC_2_EXP_FILT_MODEL_CODE)
    return boost::shared_ptr<SteeringModelBase>(new Genetic2ExpFiltModel(x,y,z));

  return boost::shared_ptr<SteeringModelBase>(new BilinearNoFiltModel(x,y,z));
}
