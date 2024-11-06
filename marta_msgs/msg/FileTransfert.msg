uint8 destination_id
string filename
int16 filelength_length
uint8 channel
bool with_ack
uint8 retry_count

#define filetransferstatus
uint8 FT_HIDLE=0
uint8 FT_RUNNING=1
uint8 FT_COMPLETED=2
uint8 FT_ABORTED=3
uint8 FT_BEGIN=4