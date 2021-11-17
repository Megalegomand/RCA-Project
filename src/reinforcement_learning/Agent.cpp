#include "Agent.h"

Agent::Agent()
{
}

Agent::Agent(State* start_state, vector<State*> state_size)
{
    current_state = start_state;
    n_states = state_size;
    
}

void Agent::take_action()
{
}

Agent::~Agent()
{
}
