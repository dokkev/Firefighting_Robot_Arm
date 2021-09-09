#ifndef AdroitCrc_h
#define  AdroitCrc_h

#include <stdint.h>
#include <vector>

//#include "hdt_common/Crc.h"

//class AdroitCrc: public Crc {
class AdroitCrc {
public:
	AdroitCrc();
	~AdroitCrc() {};
	uint32_t crc32(uint8_t *buf, uint32_t len);
	uint32_t crc32(std::vector<unsigned char> buf);
private:
	uint32_t crc_table[256];
};

#endif //  AdroitCrc_h


