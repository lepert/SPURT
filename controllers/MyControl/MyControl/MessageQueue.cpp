#include "MessageQueue.h"
#include <iostream>


MessageQueue::MessageQueue() { }
MessageQueue:: ~MessageQueue() { }

void MessageQueue::addMessageToQueue(Message msg) 
{
mq.push_back(msg);
} 

void MessageQueue::delMessageItemToQueue(Message msg, int index) 
{
mq.erase(mq.begin()+index);
}

void MessageQueue::delLastItemMessage() 
{
mq.pop_back();
}

void MessageQueue::sizeQueue() { std::cout <<  mq.size() << std::endl; }

void MessageQueue::printMessages(){
//Message tmp;
for (std::vector<Message>::iterator it = mq.begin(); it != mq.end(); ++it) {   
std::cout << (*it).getMessage() << ' ' << std::endl; 

   }
}
