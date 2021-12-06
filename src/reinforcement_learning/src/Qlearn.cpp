#include "Qlearn.h"
    

Qlearn::Qlearn()
{
}
Qlearn::Qlearn(int n_episodes_, Envoriment* states_, Agent* agent_)
{
    n_episodes = n_episodes_;
    states = states_;
    robot = agent_;

    


}
void Qlearn::doAction()
{
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
    return valid_actions[dist(rd)];

}
void Qlearn::doEpisode() //8:12
{
}
void Qlearn::train()
{
}
void Qlearn::displayQTable()
{
}
void Qlearn::implementAgent()
{
}
void Qlearn::QUpdate()
{
    //Q[state, action] = Q[state, action] + 
    // lr * (reward + gamma * np.max(Q[new_state, :]) — Q[state, action])
}
void Qlearn::ExportData()
{

}

Qlearn::~Qlearn()
{
}

