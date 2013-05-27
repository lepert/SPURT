#ifndef Message_h
#define Message_h
enum MessageType
{ 
	mtTest,
	mtGeoInform,
	mtState,
	mtRequest
	
};

// priority
enum MessagePriority
{ 
	low = 1,
	medium , 
        high   , 
        highest
		
};

class Message { 

public:

// types 
// timeStamp (!!!)
//constructor 
Message();
Message (char* _msg, int _priority, int _type);

 ~Message ();

// methods

// set-get
void setMessage (char* _msg);
char* getMessage() const;



protected:

char* msg;
int priority;
int type;


// broadcast
int broadcast;



};
#endif
