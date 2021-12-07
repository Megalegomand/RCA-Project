#include "Qlearn.h"

using namespace std;
using namespace cv;


Qlearn::Qlearn()
{
}
Qlearn::Qlearn(int n_episodes_, Envoriment* states_, Agent* agent_)
{
    n_episodes = n_episodes_;
    states = states_;
    robot = agent_;

    


}

State* Qlearn::getAction()
{
    vector<State*>valid_actions = {};
    for (int i = 0; i < robot->get_agent_location()->get_connected_states().size(); i++)
    {
        if(robot->get_agent_location()->get_connected_states()[i]->get_reward() != -8)
        {
            valid_actions.push_back(robot->get_agent_location()->get_connected_states()[i]);
        }
    }
    //srand(clock());
    //int dist = rand() % valid_actions.size();

    random_device rd;
    uniform_int_distribution<int> dist(0, valid_actions.size()-1);
    uniform_real_distribution<double> explore(0.0,1.0);

    double exploration_rate_threshold = explore(rd);

    if(epsilon < exploration_rate_threshold)
    {
        if(robot->get_agent_location()->get_VisitedCounter() == 0)
        {
            cout << "EX 1: Not visted" << endl;
            robot->get_agent_location()->set_VisitedCounter();
            //return ???? - 4:40
        }
        //Tror det skal være valid actions her er det altid 4 (men sorte bør ikke tælles med)
        else if(robot->get_agent_location()->get_VisitedCounter() <= robot->get_agent_location()->get_connected_states().size() )
        {
            if(robot->get_agent_location()->get_VisitedCounter() == robot->get_agent_location()->get_connected_states().size() )
            {
                cout << "EX 3: reset visted counter" << endl;
                robot->get_agent_location()->reset_VisitedCounter();
            }
            
            cout << "EX 2: " << endl;
            robot->get_agent_location()->set_VisitedCounter();
            // return ???? - 7:08

        }


    }
    // Skal tilpasses vores
    // 7:20 ( måske ikke alligevel)
    index_action = dist(rd);
    return valid_actions[dist(rd)];

}
State* Qlearn::doAction(Mat map)
{
    State* action = getAction();
    int x = action->get_location().first;
    int y = action->get_location().second;

   
   double current_q_value = robot->get_agent_location()->get_QValues()[index_action];
   
    // get the action and set new action
    robot->set_current_state(map,x,y);

    double reward;
    
    if (robot->get_agent_location()->get_isVisted() == true)
    {
        reward = 0;
    } 
    else
    {
        reward = robot->get_agent_location()->get_reward();
    }
    cout << "Reward: " << reward << endl;
    maxReward += reward;

    // auto max = max_element(q_table[new_sate].begin(), q_table[new_state].end())
    // double future_sa_reward = *max;
    double future_sa_reward = 1.0;

    // Q Learning equation
    double Q_value_for_state = current_q_value + lr * (reward + gamma * future_sa_reward - current_q_value);

    robot->get_agent_location()->set_QValues(index_action, Q_value_for_state);
    robot->get_agent_location()->set_isVisted();

    return robot->get_agent_location();
  
}
void Qlearn::doEpisode(Mat map)
{
 State* start_state = robot->get_starting_state();
 int steps = 0;

 int initReward = robot->get_agent_location()->get_reward();
 cout << "State reward  " << initReward << endl;
 maxReward += initReward;

 while(true)
 {
    doAction(map);
    steps++;

    if(steps == maxSteps)
    {
        cout << "Episode terminated successfully" << endl;
        cout << "Maximum Reward" << maxReward << endl;

        expectedPrEpisode.push_back(maxReward);
        maxReward = 0;
        break;
    }


 }


}
void Qlearn::train(Mat map)
{
    cout << "Bigger, better, stronger" << endl;
    for(int episode = 0; episode < n_episodes; episode++)
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
void Qlearn::QUpdate()
{
    //Q[state, action] = Q[state, action] + 
    // lr * (reward + gamma * np.max(Q[new_state, :]) — Q[state, action])
}
void Qlearn::ExportData()
{
    DataCollection.open(filename);

    for(int i = 0; i < AllEpisodes.size(); i++)
    {
        DataCollection << AllEpisodes[i] << ","; 
    }

    DataCollection << endl;

    for(int i = 0; i < expectedPrEpisode.size(); i++)
    {
        DataCollection << expectedPrEpisode[i] << ",";
    }

    DataCollection << endl;

    for(int i = 0; i < AllEpsilon.size(); i++)
    {
        DataCollection << AllEpsilon[i] << ",";
    }

    DataCollection << endl;

    for(int i = 0; i < All_lr.size(); i++)
    {
        DataCollection << All_lr[i] << ",";
    }

    DataCollection << endl;


}

Qlearn::~Qlearn()
{
}

