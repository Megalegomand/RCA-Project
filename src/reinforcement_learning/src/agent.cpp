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

State *Agent::get_current_state()
{
    return &(*envoriment->get_envoriment())[current_state.first][current_state.second];
}

void Agent::set_current_state(Mat map, int x, int y)
{
    // Recolor the old location to white
    Vec3b &old_loc = map.at<Vec3b>(get_current_state()->get_location().first, get_current_state()->get_location().second);

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
    envoriment->get_state(x, y)->set_isVisted();
    envoriment->get_state(x, y)->set_VisitedCounter();
    current_state = envoriment->get_state(x, y)->get_location();
}

void Agent::set_random_starting_state(Mat map)
{
    srand(clock());
    int x = rand() % envoriment->get_envoriment()->size();
    int y = rand() % envoriment->get_envoriment()[0].size();

    while (envoriment->get_state(x, y)->get_color_val() == Vec3b{0, 0, 0})
    {
        x = rand() % envoriment->get_envoriment()->size();
        y = rand() % envoriment->get_envoriment()[0].size();
    }

    starting_state = envoriment->get_state(x, y);
    set_current_state(map, x, y);
}

void Agent::take_action(Mat map, Action action)
{
    int x, y;

    //Stays in current state if action result in a black state (wall) othervise move
    switch (action)
    {
    case Action::UP:
        if (get_current_state()->get_connected_states()[0]->get_reward() != 0)
        {
            x = get_current_state()->get_connected_states()[0]->get_location().first;
            y = get_current_state()->get_connected_states()[0]->get_location().second;
            set_current_state(map, x, y);

            break;
        }
        else
            break;
    case Action::DOWN:
        if (get_current_state()->get_connected_states()[1]->get_reward() != 0)
        {
            x = get_current_state()->get_connected_states()[1]->get_location().first;
            y = get_current_state()->get_connected_states()[1]->get_location().second;
            set_current_state(map, x, y);

            break;
        }
        else
            break;
    case Action::LEFT:
        if (get_current_state()->get_connected_states()[2]->get_reward() != 0)
        {
            x = get_current_state()->get_connected_states()[2]->get_location().first;
            y = get_current_state()->get_connected_states()[2]->get_location().second;
            set_current_state(map, x, y);

            break;
        }
        else
            break;
    case Action::RIGHT:

        if (get_current_state()->get_connected_states()[3]->get_reward() != 0)
        {
            x = get_current_state()->get_connected_states()[3]->get_location().first;
            y = get_current_state()->get_connected_states()[3]->get_location().second;
            set_current_state(map, x, y);

            break;
        }
        else
            break;
    }
}

State *Agent::get_starting_state()
{
    return starting_state;
}

State *Agent::get_agent_location()
{
    return get_current_state();
}

Agent::~Agent()
{
}
