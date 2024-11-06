Header header

uint8 destination_id
uint8 command
string payload
int16 payload_length
uint8 priority
uint8 message_id_type
uint8 channel
bool with_ack
uint8 retry_count


#define channel
#uint8 C_BROADCAST=0
#uint8 C_RADIO=1
#uint8 C_ACOUSTIC=2









#byte destination_id; //0 for broadcast
#byte command;
#std::string payload;
#int payload_lenght;
#byte priority;
#byte message_id_type;
#byte channel; //1 radio , 2 acoustic, 0 broadcast
#bool with_ack;
#byte retry_count;
