#ifndef TRAINERS_HPP
#define TRAINERS_HPP
#include "kalman.h"
#include "beliefState.h"
 
extern void start_goToPointTrainer(Strategy::Kalman &, Strategy::BeliefState &, bool &run);
extern void start_TacticTester    (Strategy::Kalman &, Strategy::BeliefState &, bool &run);
extern void start_goToBallTrainer (Strategy::Kalman &, Strategy::BeliefState &, bool &run);
extern void train_scrap           (Strategy::Kalman &, Strategy::BeliefState &, bool &run);
#endif