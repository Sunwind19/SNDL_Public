#include "Config.h"

namespace hexabot {

const float HOME_X[LEG_COUNT] = { 82.0f,   0.0f, -82.0f, -82.0f,    0.0f,  82.0f };
const float HOME_Y[LEG_COUNT] = { 82.0f, 116.0f,  82.0f, -82.0f, -116.0f, -82.0f };
const float HOME_Z[LEG_COUNT] = { -80.0f, -80.0f, -80.0f, -80.0f, -80.0f, -80.0f };

const float BODY_X[LEG_COUNT] = { 110.4f, 0.0f, -110.4f, -110.4f,   0.0f, 110.4f };
const float BODY_Y[LEG_COUNT] = {  58.4f, 90.8f,  58.4f,  -58.4f, -90.8f, -58.4f };
const float BODY_Z[LEG_COUNT] = {   0.0f,  0.0f,   0.0f,    0.0f,   0.0f,   0.0f };

const int8_t COXA_CAL[LEG_COUNT]  = {  2, -1, -1, -3, -2, -3 };
const int8_t FEMUR_CAL[LEG_COUNT] = {  4, -2,  0, -1,  0,  0 };
const int8_t TIBIA_CAL[LEG_COUNT] = {  0, -3, -3, -2, -3, -1 };

}  // namespace hexabot
