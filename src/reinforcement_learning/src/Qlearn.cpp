#include "Qlearn.h"
    

Qlearn::Qlearn()
{
}
Qlearn::Qlearn(int n_episodes_, Envoriment* states_)
{
    n_episodes = n_episodes_;
    states = states_;


}
void Qlearn::doAction()
{
}
void Qlearn::getAction()
{
}
void Qlearn::doEpisode()
{
}
void Qlearn::train()
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

