#include "Qlearn.h"
    

Qlearn::Qlearn()
{
}
Qlearn::Qlearn(Envoriment* states_, Agent* agent_)
{
    states = states_;
    agent = agent_;
}
void Qlearn::QBellmanEq()
{
}
void Qlearn::QUpdate()
{
    //Q[state, action] = Q[state, action] + 
    // lr * (reward + gamma * np.max(Q[new_state, :]) — Q[state, action])
}
Qlearn::~Qlearn()
{
}

