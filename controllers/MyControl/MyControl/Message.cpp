#include "Message.h"0
#include <iostream>

Message::Message () { }
Message::Message (char* _msg, int _priority, int _type)
								:msg(_msg),priority(_priority), type(_type) 
{ 
}
Message::~Message () 
{ 

}


void Message::setMessage (char* _msg) 
{ 
	_msg = msg; 
}
char* Message::getMessage() const 
{ 
	return this -> msg; 
}


