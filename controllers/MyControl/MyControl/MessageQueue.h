#ifndef MessageQueue_h
#define MessageQueue_h
#include <cstdio>
#include <vector>
#include "Message.h"


class MessageQueue {

public:

MessageQueue();
~MessageQueue();	

std::vector<Message> mq;
         
// message to queue
void addMessageToQueue(Message msg);
void delMessageItemToQueue(Message msg, int index);
void delLastItemMessage();
void sizeQueue();
void printMessages();

};
#endif
