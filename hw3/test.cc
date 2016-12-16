#include "mdp-simulation.h"
#include <iostream>
#include <random>
#include <algorithm>

std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
std::uniform_int_distribution<int> uni(0, MAX_GRID-1); 
std::uniform_real_distribution<double> eps_dist(0.0,1.0);
std::uniform_int_distribution<int> action_dist(0, 3);   //for randomly choosing actions

void PrintPolicy(double Q_table[][4])
{
    for (int r=0; r < MAX_GRID; r++) {

        for (int c=0; c < MAX_GRID; c++) {

            int q_row = (r*MAX_GRID + c);
            auto max_it = std::max_element(Q_table[q_row], Q_table[q_row]+4);

            switch(max_it - Q_table[q_row]) {
                case 0: printf("N "); break;
                case 1: printf("S "); break;
                case 2: printf("E "); break;
                case 3: printf("W "); break;

            }
        }
        printf("\n");
    }

}



int main (void)
{
    double alpha = 0.05;  //learning rate
    double gamma = 0.9;  //discount factor
    double eps = 1.0;    //probability of choosing random action

    int n_episodes = 100000;
    int steps_per_episode = 100;

    /*
     * The 100 states are unrolled to one dimension.
     * it goes (top to bottom): (0,0)...(0,9),(1,0)...(1,9)...
     */
    double Q_table [MAX_GRID*MAX_GRID][4] = {0};
    int action;
    int q_row;

    State state;

    double epReward = 0;

    for (int n = 0; n < n_episodes; n++) {
        //start at a random state
        state.x = uni(rng); state.y = uni(rng);
        epReward = 0;

        for (int s = 0; s < steps_per_episode; s++) {

            //select an action greedily
            if (eps_dist(rng) > eps) {

                //convert state coordinates to q-table row index
                q_row = (state.x*MAX_GRID) + (state.y);
                auto max_it = std::max_element(Q_table[q_row], Q_table[q_row]+4);
                action = max_it - Q_table[q_row];

            } else {
                action = action_dist(rng);
            }

            State nextState = my_next_state(state, static_cast<Action>(action));
            Reward reward = my_reward(state);

            epReward += reward;

            //update q-table
            int new_q_row = (nextState.x*MAX_GRID) + (nextState.y);
            double best_q_succ = *std::max_element(Q_table[new_q_row], Q_table[new_q_row]+4);
            Q_table[q_row][action] = (1-alpha)*Q_table[q_row][action] + 
                                      alpha*(reward + gamma*best_q_succ);

            state.x = nextState.x; state.y = nextState.y;
        }
        //printf("%f\n", epReward);

        //decay epsilon
        eps = 1 - double(n) / n_episodes;
    }

    // for (int r=0; r < MAX_GRID*MAX_GRID; r++) {
    //     for (int c=0; c < 4; c++) {
    //         printf("%f ", Q_table[r][c]);
    //     }
    //     printf("\n");
    // }

    PrintPolicy(Q_table);

    return 0;
}
