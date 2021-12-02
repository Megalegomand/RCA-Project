#include "Qlearn.h"
    

Qlearn::Qlearn()
{
}
Qlearn::Qlearn(Envoriment* states, Agent* agent)
{
    for (int i = 0; i < states->get_envoriment()->size(); i++)
    {
        for (int y = 0; y < states->get_envoriment()[0].size(); y++)
        {
        
        }
        
    }
    


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

