#ifndef CAN_FRAME_STREAM
#define CAN_FRAME_STREAM

#include <mcp2515.h>

class can_frame_stream {
  static constexpr int buffsize = 20; //space for 10 can_messages - increase if needed
  can_frame cf_buffer[buffsize];
  int read_index; //where to read next message
  int write_index; //where to write next message
  bool write_lock; //buffer full
public:
  can_frame_stream() : read_index{0}, write_index{0}, write_lock{false} {};
  byte put(can_frame &frame);
  byte get(can_frame &frame);
};

#endif // CAN_FRAME_STREAM
