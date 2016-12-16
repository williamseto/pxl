// Students should just call my_next_state and my_reward
// e.g., my_next_state(State(0, 1), NORTH)
// e.g., my_reward(State(0, 1))

#ifndef _mdp_simulation_h
#define _mdp_simulation_h

// Size of the square grid
#define MAX_GRID (10)

struct State {
  State(int x, int y) : x(x), y(y) {}
  State() {}

  bool operator==(const State &state) { return x == state.x && y == state.y; }

  int x, y;
};

typedef double Reward;

enum Action { N, S, E, W };

State my_next_state (const State &myState, Action myAction);

Reward my_reward (const State &myState);

#endif // _mdp_simulation_h
