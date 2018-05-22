
struct t_sportData
{
	struct t_sportData *next ;
	uint8_t data[7] ;
	uint8_t dataLock ;
	uint8_t serialSent ;
} ;

void setNewData( struct t_sportData *pdata, uint16_t id, uint32_t value ) ;
void initSportUart() ;
void pollSport( void ) ;
void sendHubData( void ) ;
void sendHubPacket( void ) ;

extern uint8_t Sending ;

