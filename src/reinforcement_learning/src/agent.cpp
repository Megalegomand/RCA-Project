#include "agent.h"

Agent::Agent()
{
}

Agent::Agent(Envoriment *map)
{
    //current_state = start_state;ca
    n_states = map->get_envoriment()->size();
    
}
 
void Agent::take_action()
{

}

float Agent::set_exploration_proba()
{
    return exploration_proba = exploration_proba * exp(-exploration_proba);
}

float Agent::get_lr()
{
    return lr;
}

float Agent::get_gamma()
{
    return gamma;
}

float Agent::get_exploration_proba()
{
    return exploration_proba;
}

float Agent::get_exploration_proba_decay()
{
    return exploration_proba_decay;
}

float Agent::get_batch_size()
{
    return batch_size;
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
