#include "Qlearn.h"

using namespace std;
using namespace cv;

Qlearn::Qlearn()
{
}

Qlearn::Qlearn(int n_episodes_, Envoriment *states_, Agent *agent_)
{
    n_episodes = n_episodes_;
    states = states_;
    robot = agent_;
    maxReward =0.0;
}

State *Qlearn::getAction()
{
    vector<State *> valid_actions = {};
    valid_actions.clear();
    for (int i = 0; i < robot->get_agent_location()->get_connected_states().size(); i++)
    {
        if (robot->get_agent_location()->get_connected_states()[i]->get_reward() != 0)
        {
            valid_actions.push_back(robot->get_agent_location()->get_connected_states()[i]);
        }
    }
    //srand(clock());
    //int dist = rand() % valid_actions.size();

    random_device rd;
    uniform_int_distribution<int> dist(0, valid_actions.size() - 1);
    uniform_real_distribution<double> explore(0.0, 1.0);

    double exploration_rate_threshold = explore(rd);
    // cout << epsilon << endl;
    // cout << exploration_rate_threshold << endl;

    if (epsilon < exploration_rate_threshold)
    {
        // cout << epsilon << endl;
        // cout << exploration_rate_threshold << endl;
        if (robot->get_agent_location()->get_VisitedCounter() > 1)
        {
            cout << "EX 1: Not visted" << endl;
            robot->get_agent_location()->set_VisitedCounter();

            return robot->get_agent_location()->best_choice();
        }
        //Tror det skal være valid actions her er det altid 4 (men sorte bør ikke tælles med)
        else if (robot->get_agent_location()->get_VisitedCounter() <= valid_actions.size())
        {
            if (robot->get_agent_location()->get_VisitedCounter() == valid_actions.size())
            {
                cout << "EX 3: reset visted counter" << endl;
                robot->get_agent_location()->reset_VisitedCounter();
            }

            int element = robot->get_agent_location()->get_VisitedCounter();
            cout << "Element: " << element << endl;

            int largestIndex = get_largestIndex(element, valid_actions);
            robot->get_agent_location()->set_VisitedCounter();
            cout << "EX 2: Used largest index " << endl;

            return robot->get_agent_location()->get_connected_states()[largestIndex];

        }
    }
    index_action = dist(rd);
    return valid_actions[dist(rd)];
}

State *Qlearn::doAction(Mat map)
{
    int x = getAction()->get_location().first;
    int y = getAction()->get_location().second;

    double current_q_value = robot->get_agent_location()->get_QValues()[index_action];
    // get the action and set new action
    robot->set_current_state(map, x, y);
    cout << "x: " << robot->get_agent_location()->get_location().first << endl;
    cout << "y: " << robot->get_agent_location()->get_location().second << endl;
    double reward;


    if (robot->get_agent_location()->get_VisitedCounter() > 1)
    {
        reward = 0;
    }
    else
    {
        reward = robot->get_agent_location()->get_reward();
    }
    cout << "Reward: " << reward << endl;
    maxReward += reward;

    //vect-le future_sa_reward = *max;
    double future_sa_reward =1.0;

    // Q Learning equation
    double Q_value_for_state = current_q_value + lr * (reward + gamma * future_sa_reward - current_q_value);

    robot->get_agent_location()->set_QValues(index_action, Q_value_for_state);
    robot->get_agent_location()->set_isVisted();

    return robot->get_agent_location();
}

int Qlearn::get_largestIndex(int ele, vector<State *> set_of_valid_actions)
{
    double eleNumber;
    vector<double> temp = robot->get_agent_location()->get_QValues();
    sort(temp.begin(), temp.end());

    eleNumber = temp[ele];
    temp.clear();
    for(int i = 0; i < set_of_valid_actions.size(); i++)
    {
        if(robot->get_agent_location()->get_QValues()[i] == double(eleNumber))
        {
            return i;
        }
    }
    return -1;
}

void Qlearn::doEpisode(Mat map)
{
    State *start_state = robot->get_starting_state();
    int steps = 0;

    int initReward = robot->get_agent_location()->get_reward();
    cout << "State reward:  " << initReward << endl;
    maxReward += initReward;

    while (true)
    {
        doAction(map);
        steps++;

        if (steps == maxSteps)
        {
            cout << "Episode terminated successfully" << endl;
            cout << "Maximum Reward: " << maxReward << endl;

            expectedPrEpisode.push_back(maxReward);
            maxReward = 0;
            break;
        }
    }
}

void Qlearn::train(Mat map)
{
    cout << "Bigger, better, stronger" << endl;
    for (int episode = 0; episode < n_episodes; episode++)
    {
        AllEpisodes.push_back(episode);
        cout << "Episode number: " << episode << endl;

        robot->set_random_starting_state(map);
        states->reset_map(map);

        doEpisode(map);

        AllEpisodes.push_back(epsilon);
        All_lr.push_back(lr);

        epsilon = min_exploration_rate + (max_exploration_rate - min_exploration_rate) * exp(-epsilon_decay * episode);
    }

    cout << "Training finished" << endl;
    ExportData();
}

void Qlearn::displayQTable()
{
}

void Qlearn::implementAgent(Mat map)
{
    cout << "Implementing on Agent" << endl;
    epsilon = 0;
    robot->set_random_starting_state(map);
    states->reset_map(map);
    doEpisode(map);
}

void Qlearn::ExportData()
{
    DataCollection.open(filename);

    for (int i = 0; i < AllEpisodes.size(); i++)
    {
        DataCollection << AllEpisodes[i] << ",";
    }

    DataCollection << endl;

    for (int i = 0; i < expectedPrEpisode.size(); i++)
    {
        DataCollection << expectedPrEpisode[i] << ",";
    }

    DataCollection << endl;

    for (int i = 0; i < AllEpsilon.size(); i++)
    {
        DataCollection << AllEpsilon[i] << ",";
    }

    DataCollection << endl;

    for (int i = 0; i < All_lr.size(); i++)
    {
        DataCollection << All_lr[i] << ",";
    }

    DataCollection << endl;
}

Qlearn::~Qlearn()
{
}
