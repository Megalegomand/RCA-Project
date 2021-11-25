#include "agent.h"

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

float Agent::set_exploration_proba()
{
    return exploration_proba = exploration_proba * exp(-exploration_proba);
}

// void Agent::store_episode()
// {
//     memory.end({current_state,"action",
//                 State*->get_reward(),
//                 next_state,
//                 ended}); // mangler at den ved hvor den er "current_state"
// }

void Agent::train()
{

}

Agent::~Agent()
{
}
