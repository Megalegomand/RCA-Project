#include "agent.h"

Agent::Agent()
{
}

Agent::Agent(Envoriment *map)
{
    //current_state = start_state;
    envoriment = map;
    n_states = envoriment->get_envoriment()->size();
}
void Agent::set_current_state(Mat map, int x, int y)
{
    // Recolor the old location to white
    Vec3b &old_loc = map.at<Vec3b>(current_state->get_location().first, current_state->get_location().second);

    old_loc[0] = 255;
    old_loc[1] = 255;
    old_loc[2] = 255;

    map.at<Vec3b>(x, y) = old_loc;

    // Recolor the new location to indicate the position of the agent
    Vec3b &new_loc = map.at<Vec3b>(x, y);

    new_loc[0] = agent_color[0];
    new_loc[1] = agent_color[1];
    new_loc[2] = agent_color[2];

    map.at<Vec3b>(x, y) = new_loc;
    current_state = envoriment->get_state(x, y);
}

void Agent::set_random_starting_state(Mat map)
{
    srand(clock());
    int x = rand() % envoriment->get_envoriment()->size();
    int y = rand() % envoriment->get_envoriment()[0].size();

    while (envoriment->get_state(x, y)->get_reward() == 0)
    {
        int x = rand() % envoriment->get_envoriment()->size();
        int y = rand() % envoriment->get_envoriment()[0].size();
    }
    set_current_state(map, x, y);
}

void Agent::take_action(Mat map, Action action)
{
    int x,y;

    switch (action)
    {
    case Action::UP:

        x = current_state->get_connected_states()[0]->get_location().first;
        y = current_state->get_connected_states()[0]->get_location().second;
        set_current_state(map, x, y);

        break;
    case Action::DOWN:

        x = current_state->get_connected_states()[1]->get_location().first;
        y = current_state->get_connected_states()[1]->get_location().second;
        set_current_state(map, x, y);

        break;
    case Action::LEFT:

        x = current_state->get_connected_states()[2]->get_location().first;
        y = current_state->get_connected_states()[2]->get_location().second;
        set_current_state(map, x, y);

        break;
    case Action::RIGHT:

        x = current_state->get_connected_states()[3]->get_location().first;
        y = current_state->get_connected_states()[3]->get_location().second;
        set_current_state(map, x, y);

        break;
    }
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

// void Agent::train(Qlearn* algo, int number_of_episode, int interations_pr_epi)
// {
    
//     number_of_episode;
//     interations_pr_epi;
    
//     for (int i = 0; i < number_of_episode; i++)
//     {
        
//     current_state;
    
//     // // sum the rewards that the agent gets from the environment
//     int total_episode_reward = 0;
    
//     for (int i = 0; i < interations_pr_epi; i++)
//     {
//         // //  if the sampled flaot is less than the exploration proba
//         // //  we sample a float from a uniform distribution over 0 and 1
//         // //      the agent selects arandom action
//         // // else
//         // //     he exploits his knowledge using the bellman equation 
        
//         if (random.uniform(0,1) < exploration_proba)
//         {
//             action = env.action_space.sample()
//         }
            
//         else:
//             action = np.argmax(Q_table[current_state,:])
        
//         // // The environment runs the chosen action and returns
//         ////  the next state, a reward and true if the epiosed is ended.
//         next_state, reward, done, _ = env.step(action)
        
//         //  // We update our Q-table using the Q-learning iteration
//         Q_table[current_state, action] = (1-lr) * Q_table[current_state, action] +lr*(reward + gamma*max(Q_table[next_state,:]))
//         total_episode_reward = total_episode_reward + reward
        
//         // // If the episode is finished, we leave the for loop
//         if done:
//             break
//         current_state = next_state
//     // // update the exploration proba using exponential decay formula 
//     exploration_proba = max(min_exploration_proba, np.exp(-exploration_decreasing_decay*e))
//     rewards_per_episode.append(total_episode_reward)

//     } 
//     }
// }

Agent::~Agent()
{
}
